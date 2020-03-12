import numpy as np

def trunc(a,thresh):
    dec_a = a % 1
    int_a = a//1

    if dec_a % thresh < thresh/100:
        trunc_a = int_a + dec_a
    elif dec_a < thresh:
        if dec_a < thresh-dec_a:
            trunc_a = int_a
        else:
            trunc_a = int_a+thresh
    else: 
        for val in np.arange(thresh,1,thresh):
            if(dec_a-val)<=thresh:
                print(val)
                if abs(dec_a-(val)) < abs(dec_a-(val+thresh)):
                    trunc_a = int_a+val
                else:
                    trunc_a = int_a+(val+thresh)
                break

    return trunc_a

if __name__ == "__main__":
    x = trunc(3.26,.1)
    print(x)