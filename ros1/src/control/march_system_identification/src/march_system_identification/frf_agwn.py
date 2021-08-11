#!/usr/bin/env python

# FRF_AGWN : Frequency Response function with additive gaussian white
# noise for system identification

import numpy as np

def main():

    mean = 0
    std = 1 
    num_samples = 8
    # amp = 1
    samples = np.random.normal(mean, std, size=num_samples)
    # samples = amp * (samples/4.5)

    # print(samples)
    # print(np.mean(samples))
    # print(np.abs(samples) - mean(samples))
    print(np.max(samples))
    print(np.min(samples))

    return 0


if __name__== '__main__':
    print("executing main() in frf_agwn")
    main()
