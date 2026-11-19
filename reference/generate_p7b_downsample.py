"""Real pretrained ResNet18 layer2.0: independent full-tensor integer golden.
No HDL tools. Requires numpy, torch, torchvision and Pillow.
"""
import argparse
import hashlib
import json
import platform
from pathlib import Path
import numpy as np
import torch
import torchvision
from PIL import Image
from torchvision.models import ResNet18_Weights, resnet18

WEIGHT_URL = "https://download.pytorch.org/models/resnet18-f37072fd.pth"
IMAGE_URL = "https://raw.githubusercontent.com/pytorch/hub/master/images/dog.jpg"
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()


def round_away(x):
    return (np.sign(x) * np.floor(np.abs(x) + 0.5)).astype(np.int64)


def fixed(x):
    return np.sign(x) * ((np.abs(x) + (1 << 23)) >> 24)


def conv(x, w, stride, padding):
    kh, kw, _, cout = w.shape
    h = (x.shape[0] + 2*padding-kh)//stride+1
    width = (x.shape[1] + 2*padding-kw)//stride+1
    padded = np.pad(x.astype(np.int64), ((padding,padding),(padding,padding),(0,0)))
    y = np.zeros((h,width,cout),dtype=np.int64)
    for ky in range(kh):
        for kx in range(kw):
            y += padded[ky:ky+h*stride:stride,kx:kx+width*stride:stride] @ w[ky,kx]
    xt = torch.from_numpy(x.transpose(2,0,1).copy()).double()[None]
    wt = torch.from_numpy(w.transpose(3,2,0,1).copy()).double()
    other = torch.nn.functional.conv2d(xt,wt,stride=stride,padding=padding)[0].permute(1,2,0).numpy()
    assert np.array_equal(y,other)
    assert y.min() >= -(1 << 31) and y.max() < (1 << 31)
    return y


def emit(directory, name, values, bits):
    values=np.asarray(values).reshape(-1)
    path=directory/(name+'.mem')
    masked=[int(v)&((1 << bits)-1) for v in values]
    path.write_text(''.join(f'{v:0{bits//4}x}\n' for v in masked),encoding='ascii')
    assert masked==[int(v,16) for v in path.read_text().split()]
    return {'bits':bits,'elements':len(masked),'sha256':sha(path)}


def scalar_post(acc,bias,mult,skip,residual,relu):
    p=(int(acc)+int(bias))*int(mult)
    r=(abs(p)+(1 << 23))//(1 << 24)
    if p<0: r=-r
    r+=int(skip) if residual else 0
    if relu: r=max(0,min(127,r))
    assert -(1 << 31)<=r<(1 << 31)
    return r


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cache',type=Path,required=True)
    p.add_argument('--out',type=Path,default=Path(__file__).parent/'p7b_layer2_0')
    p.add_argument('--download',action='store_true')
    a=p.parse_args();a.cache.mkdir(parents=True,exist_ok=True);a.out.mkdir(parents=True,exist_ok=True)
    cp=a.cache/'resnet18-f37072fd.pth';photo=a.cache/'dog.jpg'
    for file,url in [(cp,WEIGHT_URL),(photo,IMAGE_URL)]:
        if not file.exists():
            if not a.download: raise FileNotFoundError(f'{file}; supply cached assets or --download')
            torch.hub.download_url_to_file(url,str(file),progress=False)
    assert sha(cp)=='f37072fd47e89c5e827621c5baffa7500819f7896bbacec160b1a16c560e07ec'
    torch.set_num_threads(4);torch.manual_seed(0)
    model=resnet18(weights=None).eval()
    model.load_state_dict(torch.load(cp,map_location='cpu',weights_only=True))
    image=ResNet18_Weights.IMAGENET1K_V1.transforms()(Image.open(photo).convert('RGB'))[None]
    block=model.layer2[0]
    with torch.no_grad():
        x=model.layer1(model.maxpool(model.relu(model.bn1(model.conv1(image)))))
        mid=block.relu(block.bn1(block.conv1(x)))
        skip=block.downsample(x)
        yf=block(x)
    hwc=lambda t:t[0].permute(1,2,0).numpy().astype(np.float64)
    xf,mf,sf,yfp=map(hwc,(x,mid,skip,yf))
    sx=float(xf.max()/127);sm=float(mf.max()/127);sy=float(yfp.max()/127)
    qx=np.clip(round_away(xf/sx),0,127)
    files={'input.mem':emit(a.out,'input',qx,8)}
    specs=[(block.downsample[0],block.downsample[1],x,qx,sx,sy,2,0),
           (block.conv1,block.bn1,x,qx,sx,sm,2,1),
           (block.conv2,block.bn2,mid,None,sm,sy,1,1)]
    info=[];qmid=None;qskip=None
    for mode,(cv,bn,fp_in,qin,si,so,stride,padding) in enumerate(specs):
        if mode==2: qin=qmid
        factor=bn.weight.detach().double()/torch.sqrt(bn.running_var.double()+bn.eps)
        wf=cv.weight.detach().double()*factor[:,None,None,None]
        bf=bn.bias.detach().double()-factor*bn.running_mean.double()
        with torch.no_grad():
            fusion=float((bn(cv(fp_in)).double()-torch.nn.functional.conv2d(fp_in.double(),wf,bf,stride=stride,padding=padding)).abs().max())
        assert fusion<1e-4
        w=wf.numpy().transpose(2,3,1,0)
        sw=np.max(np.abs(w),axis=(0,1,2))/127
        assert np.all(sw>0)
        qw=np.clip(round_away(w/sw),-127,127)
        b=round_away(bf.numpy()/(si*sw));mult=round_away(si*sw/so*(1 << 24))
        assert np.all((mult>0)&(mult<(1 << 31))) and np.max(np.abs(b))<(1 << 31)
        acc=conv(qin,qw,stride,padding)
        product=(acc+b)*mult
        assert np.max(np.abs(product))<(1 << 62)
        branch=fixed(product)
        if mode==0:
            out=branch;qskip=out
            assert np.any(qskip<0) and np.any(qskip>0), 'signed shortcut coverage'
        elif mode==1:
            out=np.clip(branch,0,127);qmid=out
        else:
            out=np.clip(branch+qskip,0,127)
            prefix=conv(qin[:,:,:64],qw[:,:,:64,:],1,1)
            files['prefix2.mem']=emit(a.out,'prefix2',prefix,32)
            skip_changed=int(np.count_nonzero(out!=np.clip(branch,0,127)))
            premature_relu_changed=int(np.count_nonzero(out!=np.clip(branch+np.maximum(qskip,0),0,127)))
            assert skip_changed>0 and premature_relu_changed>0
        assert out.min()>=-(1 << 31) and out.max()<(1 << 31)
        for name,values,bits in [(f'weight{mode}',qw,8),(f'bias{mode}',b,32),(f'mult{mode}',mult,32),
                                 (f'acc{mode}',acc,32),(f'output{mode}',out,32)]:
            files[name+'.mem']=emit(a.out,name,values,bits)
        info.append({'mode':mode,'stride':stride,'padding':padding,'weight_shape_hwio':list(qw.shape),
                     'weight_scales':sw.tolist(),'fusion_max_abs_error':fusion,
                     'acc_range':[int(acc.min()),int(acc.max())],'out_range':[int(out.min()),int(out.max())],
                     'out_sum':int(out.sum()),'negative_outputs':int(np.count_nonzero(out<0))})
    cases=[(-1,0,1 << 23,0,0,0),(-4,0,1 << 24,6,1,1),
           (4,0,1 << 24,-6,1,1),(140,0,1 << 24,-20,1,1),
           (-140,0,1 << 24,200,1,1),(0,-17,1 << 24,999,0,0),
           ((1 << 31)-1,(1 << 31)-1,(1 << 31)-1,-2147483648,1,1),
           (-(1 << 31),-(1 << 31),(1 << 31)-1,2147483647,1,1)]
    rng=np.random.default_rng(705)
    while len(cases)<256:
        mode=len(cases)%3
        cases.append((int(rng.integers(-40000,40001)),int(rng.integers(-1000,1001)),
                      int(rng.integers(0,1 << 24)),int(rng.integers(-1000,1001)),mode==2,mode!=0))
    words=[]
    for acc,b,m,s,res,relu in cases:
        value=scalar_post(acc,b,m,s,res,relu)
        fields=[(int(res)|(int(relu)<<1),8),(s,32),(m,32),(b,32),(acc,32),(value,32)]
        packed=0
        for v,bits in fields: packed=(packed << bits)|(int(v)&((1 << bits)-1))
        words.append(f'{packed:042x}\n')
    vp=a.out/'post_vectors.mem';vp.write_text(''.join(words),encoding='ascii')
    files[vp.name]={'bits':168,'elements':len(words),'sha256':sha(vp)}
    metrics={'mae':float(np.mean(np.abs(out*sy-yfp))),
             'relative_l2':float(np.linalg.norm(out*sy-yfp)/np.linalg.norm(yfp))}
    manifest={'block':'ResNet18 layer2.0','source_model':'torchvision IMAGENET1K_V1',
              'weights_url':WEIGHT_URL,'weights_sha256':sha(cp),'image_url':IMAGE_URL,'image_sha256':sha(photo),
              'input_source':'FP32 stem + full layer1 on real image; quantized at layer2.0 boundary',
              'input_hwc':[56,56,64],'output_hwc':[28,28,128],
              'scales':{'input':sx,'conv1_output':sm,'shared_main_shortcut_output':sy},
              'rounding':'nearest, ties away from zero; Q24 nonnegative multiplier; zero point 0',
              'shortcut':'signed INT32 at output scale; NO ReLU or INT8 clipping before add',
              'calibration':'single image functional validation only; not ImageNet accuracy',
              'software_checks':'full NumPy INT64 == PyTorch exact-double convolutions; BN fusion and mem round trips checked',
              'hardware_status':'NOT RUN; user runs Vivado',
              'versions':{'python':platform.python_version(),'numpy':np.__version__,'torch':torch.__version__,'torchvision':torchvision.__version__},
              'generator_sha256':sha(__file__),'layers':info,
              'skip_changed_outputs':skip_changed,'incorrect_shortcut_relu_changed_outputs':premature_relu_changed,
              'quantized_vs_float':metrics,'files':files}
    (a.out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'software_only':True,'files':len(files),'skip_negative_count':info[0]['negative_outputs'],
                      'skip_changed_outputs':skip_changed,'incorrect_shortcut_relu_changed_outputs':premature_relu_changed,
                      'quantized_vs_float':metrics},indent=2))


if __name__=='__main__':main()
