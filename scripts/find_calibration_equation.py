import matplotlib.pyplot as plt
import numpy as np


def filter_stat_data(x, y):
    new_x = []
    new_y = []
    for k in range(0, (len(x) - 1)):
        if x[k] > 12000:
            new_x.append(x[k])
            new_y.append(y[k])
    return new_x, new_y


def fit_data(x, y):
    return np.polyfit(y, x, 1)


def find_error(a, b, x, y):
    y_fit = np.multiply(a, x) + b
    return np.abs(y_fit - y)


def remove_outliers(err, x, y):
    new_x = []
    new_y = []
    mean = np.mean(err)
    sd = np.std(err)
    z = (err - mean)/sd
    for i in range(0, len(z)):
        if abs(z[i]) < 2:
            new_x.append(x[i])
            new_y.append(y[i])
    return new_x, new_y


def plot_data(a, b, y, x):
    y_fit = np.multiply(a, x) + b
    plt.plot(x, y_fit, 'b')
    plt.plot(x, y, 'ro')
    plt.xlabel('ADC')
    plt.ylabel('force')
    plt.show()


arr_a_x = []
arr_b_x = []
arr_a_y = []
arr_b_y = []


if __name__ == '__main__':

    positions = np.loadtxt('calibration_data/positions.txt')
    for i in range(1, 3):

        # 1st and 2nd column data from file are x-direction data
        # 3rd and 4th column are y-direction data
        data_x = np.loadtxt('calibration_data/{}.txt'.format(i), usecols=(0, 1))
        data_y = np.loadtxt('calibration_data/{}.txt'.format(i), usecols=(2, 3))

        force_x = data_x[:, 0]
        adc_x = data_x[:, 1]
        force_y = data_y[:, 0]
        adc_y = data_y[:, 1]

        # Find parameters for linear equation from data fitting
        new_adc_x, new_force_x = filter_stat_data(adc_x, force_x)
        new_adc_y, new_force_y = filter_stat_data(adc_y, force_y)
        # Find parameters for linear equation from data fitting
        est_a_x, est_b_x = np.polyfit(new_adc_x, new_force_x, 1)
        est_a_y, est_b_y = np.polyfit(new_adc_y, new_force_y, 1)
        # find difference between fitted line and data
        err_x = find_error(est_a_x, est_b_x, new_adc_x, new_force_x)
        err_y = find_error(est_a_y, est_b_y, new_adc_y, new_force_y)
        # remove errors from data that are more than
        adc_x, force_x = remove_outliers(err_x, new_adc_x, new_force_x)
        adc_y, force_y = remove_outliers(err_y, new_adc_y, new_force_y)
        # Find parameters after outliers removal
        a_x, b_x = fit_data(adc_x, force_x)
        a_y, b_y = fit_data(adc_y, force_y)

        plot_data(a_x, b_x, adc_x, force_x)
        plot_data(a_y, b_y, adc_y, force_y)

        # write data in array
        arr_a_x.append(a_x)
        arr_b_x.append(b_x)
        arr_a_y.append(a_y)
        arr_b_y.append(b_y)

    a_x_final = fit_data(positions, arr_a_x)
    a_y_final = fit_data(positions, arr_a_y)
    calib_equation = np.array(a_x_final, a_x_final)
    print 'a_x = ', a_x_final
    print 'a_y = ', a_y_final
    print calib_equation

    np.savez('calibration_equation.npz', transform=calib_equation)

    file_x = open('data_analysis_results_x.txt', 'w')
    # loop through each item in the list and write it to the output file
    for (pos, a, b) \
            in zip(positions, arr_a_x, arr_b_x):
        file_x.write('{} {} {} \n'.format(str(pos), str(a), str(b)))
    file_x.write('\n \n')
    file_x.close()

    file_y = open('data_analysis_results_y.txt', 'w')
    # loop through each item in the list and write it to the output file
    for (pos, a, b) \
            in zip(positions, arr_a_y, arr_b_y):
        file_y.write('{} {} {} \n'.format(str(pos), str(a), str(b)))
    file_y.write('\n \n')
    file_y.close()





