import os
import pandas
import numpy as np
import numpy.typing as npt

""" def decode_raw_can_data(bytestring: bytearray) -> npt.NDArray:

    return  """

if __name__ == "__main__":
    path_raw_file     = os.path.join(os.getcwd(), "stiebel_can_raw.csv")
    path_decoded_file = os.path.join(os.getcwd(), "stiebel_can_decoded.csv")

    dataframe_raw     = pandas.read_csv(path_raw_file, names=['timestamp','B0','B1','B2','B3','B4','B5','B6'])
    dataframe_decoded = pandas.read_csv(path_decoded_file)

    last_time = 0

    for index, row in dataframe_raw.iterrows():
        delta_time = row['timestamp'] - last_time
        hex_data = np.asarray(row['B0':].to_list())
        
        for position, current_value in enumerate(hex_data):
            if len(current_value) == 1:
                hex_data[position] = '0' + current_value

        hex_data = ' '.join(hex_data).lower()
        byte_data = bytearray.fromhex(''.join(hex_data))

        ########################################
        can_key    = 8 * (byte_data[0] & 240) + (byte_data[1] & 15)    # 240 -> 'F0' 15 -> '0F'
        data_key   = int.from_bytes(byte_data[-4:-2], byteorder='big')
        data_value = int.from_bytes(byte_data[-2:], byteorder='big') * 0.1

        send_status = 3
        if (byte_data[1]) == 121: # 121 -> 0x79
            send_status = 0
        if (byte_data[0] & 15) == 1:
            send_status = 1
        if (byte_data[0] & 15) == 2:
            send_status = 2
        
        if data_key == 17:
            last_time  = row['timestamp']
            delta_time = round(delta_time * 0.001, 2)

            print('-----------------------------')
            print(row['timestamp'], delta_time , hex_data)
            print(row['timestamp'], delta_time , list(byte_data))
            print(row['timestamp'], delta_time , hex(can_key), send_status, data_key, data_value)
