# FRF_AGWN : Frequency Response function with additive gaussian white
# noise for system identification

import numpy as np
from numpy.random import sample

def additive_gaussian_white_noise(mean=0.0, std=1.0, num_samples=1):

    samples = np.random.normal(mean, std, size=num_samples)

    return samples

def main():

    feedforward = additive_gaussian_white_noise()
    print(feedforward)

    return 0


if __name__ == '__main__':
    print("executing main() in frequency_response_analysis")
    main()
