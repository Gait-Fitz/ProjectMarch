from typing import List

import numpy as np
from pyriemann.estimation import Covariances
from pyriemann.utils.tangentspace import tangent_space


def convert_to_list_of_floats(line: str) -> List[float]:
    return [float(value) for value in line.rstrip('\n').split(',')]


def nmf(data, manual_scaled_weights):
    weighted_channels = np.empty((0, data.shape[1]))
    i = 0
    for value in manual_scaled_weights[:, 0]:  # CHANGE LATER: to no indexing, because 1 BV is 1d array
        if value != 0:  # CHANGE LATER: to how we apply weights and drop channels
            multiplied_values = [element * value for element in data[i]]
        else:
            multiplied_values = [element * 0.0001 for element in data[i]]
        weighted_channels = np.append(weighted_channels, [multiplied_values], axis=0)
        i = i + 1

    return weighted_channels


def feature_extraction(nmf_ata, c_ref):
    sw_cov_matrix_train = Covariances('lwf').transform(
        np.array([nmf_ata, nmf_ata]))  # how to make sure always positive definitive?

    t_train = tangent_space(sw_cov_matrix_train, c_ref)

    return t_train[0]


def ncfs(feature_extraction_data, feature_select_coefs):
    t_train_selected = []
    i = 0
    for weights in feature_select_coefs:
        if weights > 0.02 * np.max(feature_select_coefs):
            t_train_selected = np.append(t_train_selected, feature_extraction_data[i])
        i = i + 1

    return t_train_selected


def lda(ncfs_data, lda_model):
    output = lda_model.predict([ncfs_data])

    return output
#
#
# def main():
#     headset = Headset('NAUTILUS')
#
#     manual_scaled_weights = np.loadtxt('manual_scaled_weights.csv', delimiter=',')
#     Cref = np.loadtxt('Cref.csv', delimiter=',')
#     feature_select_coefs = np.loadtxt('feature_select_coefs.csv', delimiter=',')
#     LDAmodel = load('LDAmodel.pkl')
#
#     i = 0
#     while i == 0:
#         try:
#             start = timeit.default_timer()
#
#             data = headset.get_data(250)
#             data = headset.preprocessing(data)
#             NMFdata = nmf(data, manual_scaled_weights)
#             featureExtractionData = feature_extraction(NMFdata, Cref)
#             NCFSdata = ncfs(featureExtractionData, feature_select_coefs)
#             output = lda(NCFSdata, LDAmodel)
#             print(output)
#
#             end = timeit.default_timer()
#             print('Processing time 1 sec data:', end - start)  # 1.06 - 1.09 sec
#
#         except:
#             print('Error')
#             continue

#
# if __name__ == '__main__':
#     main()
