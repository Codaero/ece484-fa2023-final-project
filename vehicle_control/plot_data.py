import matplotlib.pyplot as plt
import numpy as np
import csv

###### plot trajectory #########
def plot_trajectory(file_path):
    data_array = np.loadtxt(file_path)

    x_pos = []
    y_pos = []
    theta = []
    for row in data_array:
        print(row[0])
        x_pos.append(float(row[0]))
        y_pos.append(float(row[1]))
        # theta.append(float(row[2]))

    plt.scatter(x_pos, y_pos)
    plt.scatter(x_pos[-1], y_pos[-1])
    plt.show()


############ remove duplicate rows #########
def remove_duplicate_rows_and_save(input_file_path, output_file):
    unique_rows = set()

    with open(input_file_path, 'r') as infile, open(output_file, 'w', newline='') as outfile:
        reader = csv.reader(infile, delimiter=' ')
        writer = csv.writer(outfile, delimiter=' ')

        for row in reader:
            # Convert the row to a tuple to make it hashable and add it to the set
            row_tuple = tuple(row)
            if row_tuple not in unique_rows:
                unique_rows.add(row_tuple)
                writer.writerow(row)



##################Insert commas #####################################
def insert_commas(input_file_path, output_file_name):
    # Read the input data
    with open(input_file_path, 'r') as input_file:
        # Read lines from the file
        lines = input_file.readlines()

    # Process each line to insert commas
    csv_data = [line.strip().replace(' ', ',') for line in lines]

    # Write the result to the output CSV file
    with open(output_file_name, 'w', newline='') as output_csv_file:
        csv_writer = csv.writer(output_csv_file)
        csv_writer.writerows([row.split(',') for row in csv_data])

    print(f"CSV file has been created: {output_file_name}.")

def txt_to_csv(input_txt_path, output_csv_path):
    # Read the data from the text file using numpy.genfromtxt
    data = np.genfromtxt(input_txt_path, dtype=None, encoding=None)

    # Save the data to a CSV file using numpy.savetxt
    np.savetxt(output_csv_path, data, delimiter=',', fmt='%s')

    print(f"CSV file has been created: {output_csv_path}.")




input_file_path = "/home/paulosuma/Documents/SafeAuto/mp-release-23fa-main/testfile.csv"
output_file_path = "/home/paulosuma/Documents/SafeAuto/mp-release-23fa-main/testfile_commas.csv"

# plot_trajectory(input_file_path)

# insert_commas(input_file_path, output_file_path)

remove_duplicate_rows_and_save(input_file_path, output_file_path)





