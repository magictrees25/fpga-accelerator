#!/usr/bin/env python3
"""Independent mapping/scoreboard precheck for P5-A six-run scenarios.
Not an HDL simulation. Checks local HWC and Reader pair mapping vs GLOBAL convolution.
"""
from dataclasses import dataclass

@dataclass(frozen=True)
class Mode:
    name: str
    H: int; W: int; OH: int; OW: int; KH: int; KW: int; stride: int; pad: int

modes = {
    0: Mode('3x3/S1/P1 (Line Buffer)',10,10,8,8,3,3,1,1),
    1: Mode('1x1/S2/P0 (direct BRAM)',8,8,4,4,1,1,2,0),
    2: Mode('3x3/S2/P1 (direct BRAM)',9,9,4,4,3,3,2,1),
}

def act(y,x,c,e):
    if not (0 <= y < 56 and 0 <= x < 56): return 0
    v=((17*y+11*x+7*c+37*e)%255)-127
    if (y,x,c)==(0,0,0):v=-128
    if (y,x,c)==(54,54,63):v=127
    return v

def weight(ky,kx,c,n,e):
    v=((13*ky+7*kx+3*c+5*n+41*e)%31)-15
    if (ky,kx,c,n)==(0,0,0,0):v=-128
    if c==63 and n==7 and ky in (0,2) and kx in (0,2):v=127
    return v

rows=[]
for e in range(6):
    idx=e%3; p=modes[idx]; K=p.KH*p.KW*64; M=p.OH*p.OW
    orig=0 if e<3 else (48 if idx==0 else 24)
    reqs=0; checks=0; pad_pixels=0; checksum=0
    for ly in range(p.H):
        for lx in range(p.W):
            y=orig*p.stride+ly-p.pad
            x=orig*p.stride+lx-p.pad
            if y<0 or y>=56 or x<0 or x>=56:pad_pixels+=1
    for m in range(M):
        oy,ox=divmod(m,p.OW)
        for k in range(0,K,2):
            ky,kx=divmod(k//64,p.KW);ci=k%64
            py=oy*p.stride+ky;px=ox*p.stride+kx
            local_pair=((py*p.W+px)*64+ci)//2
            if idx==0:
                ring_addr=((py%4)*p.W+px)*32+ci//2
                assert 0<=ring_addr<4*p.W*32
            elif idx==1:
                direct_addr=512*oy+64*ox+k//2
                assert direct_addr==local_pair
            elif idx==2:
                direct_addr=((2*oy+ky)*9+(2*ox+kx))*32+ci//2
                assert direct_addr==local_pair
            for lane in (0,1):
                gy=(orig+oy)*p.stride+ky-p.pad
                gx=(orig+ox)*p.stride+kx-p.pad
                local_gy=orig*p.stride+py-p.pad
                local_gx=orig*p.stride+px-p.pad
                assert gy==local_gy and gx==local_gx
                assert act(gy,gx,ci+lane,e)==act(local_gy,local_gx,ci+lane,e)
                checks+=1
            reqs+=1
        for n in range(8):
            acc=0
            for ky in range(p.KH):
                for kx in range(p.KW):
                    for ci in range(64):
                        gy=(orig+oy)*p.stride+ky-p.pad
                        gx=(orig+ox)*p.stride+kx-p.pad
                        acc += act(gy,gx,ci,e)*weight(ky,kx,ci,n,e)
            assert -(1<<31)<=acc<1<<31
            checksum+=acc
    cycles=M*(1+K//2+13+1+1+1)
    rows.append((e,p.name,M*8,reqs,cycles,pad_pixels*64,checksum))
    print(f'Run {e}: {p.name}, outputs={M*8}, K-pairs={reqs}, cycles={cycles}, pad_bytes={pad_pixels*64}, checksum={checksum}')
assert sum(r[2] for r in rows)==1536
assert sum(r[3] for r in rows)==47104
assert sum(r[4] for r in rows)==50368
print('PASS: 6 runs, 1536 outputs, 47104 pairs, 50368 compute cycles (REFERENCE ONLY)')
