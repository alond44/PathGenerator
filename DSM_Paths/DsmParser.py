import os
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import cv2

mpl.use('TkAgg')  # or can use 'TkAgg', whatever you have/prefer *couldn't run with 'Qt5Agg'...


def DSMParcer(Inputpath, Filename, SaveTIF):
    # Filename = 'dsm_binary'
    SaveName = 'DSM'
    SaveFolder = 'DSMout'
    SaveType = 'tif'
    MainFolderPath = f"{Inputpath}/"

    DsmPath = 'BinFiles/'
    # D:/Results/17_9_2020/output/oracle/
    # D:/Results/17_9_2020/out/oracle
    Filepath = f'{MainFolderPath}{DsmPath}'
    SavePath = f'{MainFolderPath}{DsmPath}{SaveFolder}'
    fullFilename = f'{Filepath}{Filename}.bin'
    if not (os.path.isdir(SavePath)):
        os.mkdir(SavePath)
    with open(fullFilename, 'rb') as DsmHandleID:
        lat_org = np.fromfile(DsmHandleID, dtype=np.float64, count=1)
        long_org = np.fromfile(DsmHandleID, dtype=np.float64, count=1)
        alt_org = np.fromfile(DsmHandleID, dtype=np.float64, count=1)
        x_org = np.fromfile(DsmHandleID, dtype=np.float32, count=1)
        y_org = np.fromfile(DsmHandleID, dtype=np.float32, count=1)
        z_org = np.fromfile(DsmHandleID, dtype=np.float32, count=1)

        Wx = np.fromfile(DsmHandleID, dtype=np.uint32, count=1)
        Wy = np.fromfile(DsmHandleID, dtype=np.uint32, count=1)

        dWx = np.fromfile(DsmHandleID, dtype=np.uint8, count=1)
        dWy = np.fromfile(DsmHandleID, dtype=np.uint8, count=1)

        NumOfPixs = Wx.item() * Wy.item()
        dsm_vec = np.fromfile(DsmHandleID, dtype=np.float32, count=NumOfPixs)
        dsm = np.reshape(dsm_vec, (Wx.item(), Wy.item()))
        if SaveTIF:
            fullSaveFile = f'{SavePath}/{SaveName}{0}.{SaveType}'
            cv2.imwrite(fullSaveFile, dsm)
        return lat_org, long_org, alt_org, x_org, y_org, z_org, Wx.item(), Wy.item(), dWx.item(), dWy.item(), dsm


def create_map(input_path, file_name, save_tif=False, to_print=False):
    lat_org, long_org, alt_org, x_org, y_org, z_org, Wx, Wy, dWx, dWy, dsm = DSMParcer(input_path, file_name, save_tif)
    if to_print:
        print(f'{lat_org},{long_org},{alt_org}')
        print(f'{x_org}{y_org}{z_org}')
        print(f'{Wx},{Wy},{dWx},{dWy}')
        plt.figure(1)
        im = plt.imshow(dsm)
        plt.show()
    return dsm

