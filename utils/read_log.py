import pybullet as p
import struct
import sys
import os
import pandas as pd
import seaborn as sns

import matplotlib.pyplot as plt
from matplotlib import rcParams


class LogReader:
    def __init__(self):
        self.log = None
        self.keys = None

    # Read the log file
    def read(self, filename, verbose=False):
        f = open(filename, "rb")

        if verbose:
            print("Opened"),
            print(filename)

        keys = f.readline().decode("utf8").rstrip("\n").split(",")
        fmt = f.readline().decode("utf8").rstrip("\n")

        # The byte number of one record
        sz = struct.calcsize(fmt)
        # The type number of one record
        ncols = len(fmt)

        if verbose:
            print("Keys:"),
            print(keys)
            print("Format:"),
            print(fmt)
            print("Size:"),
            print(sz)
            print("Columns:"),
            print(ncols)

        # Read data
        wholeFile = f.read()
        # split by alignment word
        chunks = wholeFile.split(b"\xaa\xbb")
        if verbose:
            print("num chunks")
            print(len(chunks))

        log = list()
        for chunk in chunks:
            if len(chunk) == sz:
                values = struct.unpack(fmt, chunk)
                record = list()
                for i in range(ncols):
                    record.append(values[i])
                log.append(record)

        self.log = log
        self.keys = keys

        return log, keys

    # Put the data into a pandas dataframe, keeping only the important columns
    def make_dataframe(self, columns=None):
        df = pd.DataFrame(self.log, columns=self.keys)

        if isinstance(columns, list):
            df = df[columns]

        self.df = df
        return df

    # Plot the data, and pass any extra arguments directly to seaborn
    def plot(self, data=None, **fargs):
        if data is None:
            data = self.df
        data = data.melt("timeStamp", var_name="cols", value_name="vals")
        sns.lineplot(data=data, **fargs)


if __name__ == "__main__":
    # Read the data file
    filename = "../../data/test.dat"
    reader = LogReader()
    reader.read(filename)
    reader.make_dataframe(["timeStamp", "posX", "posY", "posZ", "oriX", "oriY", "oriZ"])

    # Set up the plot nicely
    rcParams["lines.linewidth"] = 2
    rcParams["font.size"] = 14
    rcParams["font.family"] = "Arial"
    sns.set_style("ticks")
    sns.set_palette("bright")
    fig = plt.figure()

    # Plot the data
    reader.plot(x="timeStamp", y="vals", hue="cols")

    # Pretty-up the plot
    leg = plt.legend(loc="upper right")
    plt.xlabel("Time (sec)")
    plt.ylabel("Data")
    plt.tight_layout()

    # Save the plot to a file and show it
    plt.savefig(filename.replace(".dat", ".png"), dpi=450)
    plt.show()
