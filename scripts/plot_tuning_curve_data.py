#!/usr/bin/env python

import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import os
import pandas as pd
import seaborn as sns
import time

plt.switch_backend('agg')

DIR= '../../..'
FILE=f'{DIR}/tuning_curve_samples.csv'

sns.set_style('whitegrid')

def main():
    while True:
        D= pd.read_csv(FILE)
        # drop J1s but keep WRJ1. The others don't exist
        D= D[
            (D['joint'] != 'FFJ1') &
            (D['joint'] != 'MFJ1') &
            (D['joint'] != 'RFJ1') &
            (D['joint'] != 'LFJ1') &
            (D['joint'] != 'THJ1')
            ]
        try:
            g = sns.FacetGrid(D, col='joint', col_wrap=5, height=4, sharex=False, sharey=False, hue='joint')
            g.map(sns.scatterplot, 'reading', 'position', linewidth = 0)
            for ax in g.axes.flat:
                ax.xaxis.set_major_locator(plticker.MultipleLocator(100))
                # format, e.g.,  1200 as 1.2k
                ax.xaxis.set_major_formatter(plticker.FuncFormatter(lambda x, _: f'{x/1000:.1f}k'))
                # rotate labels
                for label in ax.get_xticklabels():
                    label.set_rotation(60)
                ax.yaxis.set_major_locator(plticker.MaxNLocator(20))
                ax.yaxis.set_major_formatter(plticker.FormatStrFormatter('%.2f'))


            # first save in tmp file location, then mv to final location (overwriting the previous file)
            g.savefig(f'{DIR}/tuning_curve_data.tmp.png')
            os.replace(f'{DIR}/tuning_curve_data.tmp.png', f'{DIR}/tuning_curve_data.png')
            plt.close(g.figure)
        except ValueError as e:
            print(e)
            pass

        print('.', end='', flush=True)
        time.sleep(.5)

    
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass