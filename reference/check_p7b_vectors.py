"""Software-only exported-memory / local-window / scatter audit; no HDL tools."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from generate_p7b_downsample import scalar_post


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data',type=Path,default=Path(__file__).parent/'p7b_layer2_0')
    parser.add_argument('--report',type=Path)
    a=parser.parse_args()
    meta=json.loads((a.data/'manifest.json').read_text(encoding='utf-8'))
    for name,info in meta['files'].items():
        path=a.data/name
        assert hashlib.sha256(path.read_bytes()).hexdigest()==info['sha256'],name
        words=path.read_text().split()
        assert len(words)==info['elements'],name
        assert all(len(v)==info['bits']//4 and 0<=int(v,16)<2**info['bits'] for v in words),name
    def load(name,bits):
        v=np.array([int(s,16) for s in (a.data/(name+'.mem')).read_text().split()],dtype=np.int64)
        return np.where(v&(1 << (bits-1)),v-(1 << bits),v)
    x=load('input',8).reshape(56,56,64)
    mid=shortcut=None
    totals={'tiles':0,'segments':0,'pair_feed':0,'raw_writes':0,'post_writes':0,'readbacks':0,
            'activation_bytes':0,'weight_words':0,'padding_scalars':0,'changed_cache_lanes':0,'int32_values':0}
    previous=None
    phase_summary=[]
    for mode in range(3):
        kh=1 if mode==0 else 3
        cin=128 if mode==2 else 64
        stride=1 if mode==2 else 2
        pad=0 if mode==0 else 1
        inp=mid if mode==2 else x
        weights=load(f'weight{mode}',8).reshape(kh,kh,cin,128)
        b=load(f'bias{mode}',32);mult=load(f'mult{mode}',32)
        expected_acc=load(f'acc{mode}',32).reshape(28,28,128)
        expected_out=load(f'output{mode}',32).reshape(28,28,128)
        prefix=load('prefix2',32).reshape(28,28,128) if mode==2 else None
        output=np.zeros((28,28,128),dtype=np.int64);seen=np.zeros_like(output,dtype=bool)
        padded=np.pad(inp,((pad,pad),(pad,pad),(0,0)))
        valid=np.pad(np.ones(inp.shape[:2],dtype=bool),((pad,pad),(pad,pad)))
        # Verify all exported signed 8-lane packing once per channel group/chunk.
        packed_weight={}
        for nb in range(0,128,8):
            for ct in range(cin//64):
                w=weights[:,:,ct*64:(ct+1)*64,nb:nb+8].reshape(-1,8)
                words=[sum((int(v)&255)<<(8*n) for n,v in enumerate(row)) for row in w]
                decoded=np.array([[(word>>(8*n))&255 for n in range(8)] for word in words],dtype=np.int64)
                decoded=np.where(decoded>=128,decoded-256,decoded)
                assert np.array_equal(decoded,w)
                packed_weight[nb,ct]=decoded
        for oy in range(0,28,4):
            for ox in range(0,28,4):
                patch=np.stack([padded[(oy+y)*stride:(oy+y)*stride+kh,(ox+xx)*stride:(ox+xx)*stride+kh]
                                for y in range(4) for xx in range(4)])
                pad_count=sum(np.count_nonzero(~valid[(oy+y)*stride:(oy+y)*stride+kh,(ox+xx)*stride:(ox+xx)*stride+kh])
                              for y in range(4) for xx in range(4))*cin*16
                totals['padding_scalars']+=int(pad_count)
                for nb in range(0,128,8):
                    acc=np.zeros((16,8),dtype=np.int64)
                    for ct in range(cin//64):
                        act=patch[:,:,:,ct*64:(ct+1)*64].reshape(16,-1)
                        # Linear patch addresses 0..K-1; both banks return k/k+1.
                        banks=(act[:,0::2],act[:,1::2])
                        restored=np.empty_like(act);restored[:,0::2]=banks[0];restored[:,1::2]=banks[1]
                        assert np.array_equal(restored,act)
                        acc+=restored@packed_weight[nb,ct]
                        if mode==2 and ct==0:
                            assert np.array_equal(acc,prefix[oy:oy+4,ox:ox+4,nb:nb+8].reshape(16,8))
                        totals['segments']+=16
                        totals['pair_feed']+=act.size//2
                        totals['activation_bytes']+=act.size
                        totals['weight_words']+=act.size
                    assert np.array_equal(acc,expected_acc[oy:oy+4,ox:ox+4,nb:nb+8].reshape(16,8))
                    result=np.empty_like(acc)
                    for m in range(16):
                        for n in range(8):
                            skip=shortcut[oy+m//4,ox+m%4,nb+n] if mode==2 else 0
                            result[m,n]=scalar_post(acc[m,n],b[nb+n],mult[nb+n],skip,mode==2,mode!=0)
                    assert np.array_equal(result,expected_out[oy:oy+4,ox:ox+4,nb:nb+8].reshape(16,8))
                    assert not seen[oy:oy+4,ox:ox+4,nb:nb+8].any()
                    seen[oy:oy+4,ox:ox+4,nb:nb+8]=True
                    output[oy:oy+4,ox:ox+4,nb:nb+8]=result.reshape(4,4,8)
                    if previous is not None:totals['changed_cache_lanes']+=int(np.count_nonzero(previous!=result))
                    previous=result.copy()
                    totals['tiles']+=1;totals['raw_writes']+=16;totals['post_writes']+=16
                    totals['readbacks']+=128;totals['int32_values']+=128
        assert seen.all() and np.array_equal(output,expected_out)
        if mode==0:shortcut=output
        if mode==1:mid=output
        phase_summary.append({'mode':mode,'outputs':int(output.size),'negative_outputs':int(np.count_nonzero(output<0))})
    for line in (a.data/'post_vectors.mem').read_text().split():
        v=int(line,16)
        signed=lambda w:w-(1 << 32) if w&(1 << 31) else w
        fields=[signed((v>>shift)&0xffffffff) for shift in [32,64,96,128]]
        expected=signed(v&0xffffffff)
        assert scalar_post(*fields,bool((v>>160)&1),bool((v>>161)&1))==expected
    assert totals['tiles']==2352 and totals['segments']==50176
    assert totals['pair_feed']==11239424 and totals['activation_bytes']==22478848
    report={'software_only':True,'HDL_simulation':'NOT RUN','mem_hashes_checked':len(meta['files']),
            'post_vectors_checked':256,'totals':totals,'phases':phase_summary,'final_int8_outputs':int(output.size)}
    text=json.dumps(report,indent=2)+'\n'
    if a.report:a.report.write_text(text,encoding='utf-8')
    print(text)


if __name__=='__main__':main()
