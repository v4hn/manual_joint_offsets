#!/usr/bin/env python

import cv2
import imutils
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import os
import pandas as pd
import rospy
import seaborn as sns
import time

plt.switch_backend('agg')

sns.set_style('whitegrid')

def main():
    FILE= rospy.get_param("manual_calibration/samples_file")
    # resolve full path to file and get dir name
    FILE= os.path.abspath(FILE)
    DIR= os.path.dirname(FILE)

    even_run = False

    while not rospy.is_shutdown():
        try:
            D= pd.read_csv(FILE)
        except FileNotFoundError:
            print('x', end='', flush=True)
            time.sleep(5)
            continue
        # drop J1s except for WRJ1. The biotac hand does not have these
        D= D[
            (D['joint'] != 'FFJ1') &
            (D['joint'] != 'MFJ1') &
            (D['joint'] != 'RFJ1') &
            (D['joint'] != 'LFJ1') &
            (D['joint'] != 'THJ1') &
            (D['reading'] != 0)
            ]
        try:
            g = sns.FacetGrid(D, col='joint', col_wrap=5, height=4, sharex=False, sharey=False, hue='joint')
            g.map(sns.scatterplot, 'reading', 'position', linewidth = 0)
            for ax in g.axes.flat:
                # format, e.g.,  1200 as 1.2k
                ax.xaxis.set_major_formatter(plticker.FuncFormatter(lambda x, _: f'{x/1000:.1f}k'))
                ax.xaxis.set_major_locator(plticker.MultipleLocator(100))
                for label in ax.get_xticklabels():
                    label.set_rotation(60)
                ax.yaxis.set_major_locator(plticker.MaxNLocator(20))
                ax.yaxis.set_major_formatter(plticker.FormatStrFormatter('%.2f'))


            # first save in tmp file location, then mv to final location (overwriting the previous file)
            g.savefig(f'{DIR}/tuning_curve_data.tmp.png')
            os.replace(f'{DIR}/tuning_curve_data.tmp.png', f'{DIR}/tuning_curve_data.png')
            plt.close(g.figure)

            img= cv2.imread(f'{DIR}/tuning_curve_data.png')
            img = imutils.resize(img, width=1600)
            # alternate between black and white box in bottom right corner to show the image is updating
            if even_run:
                img[-20:, -20:, :] = 0
            else:
                img[-20:, -20:, :] = 255
            even_run = not even_run
            cv2.imshow('tuning_curve_data', img)
            cv2.waitKey(1)
        except ValueError as e:
            print(e)
            pass

        print('.', end='', flush=True)
        time.sleep(.5)

    
if __name__ == '__main__':
    try:
        rospy.init_node("plot_tuning_curve_data")
        main()
    except KeyboardInterrupt:
        pass
