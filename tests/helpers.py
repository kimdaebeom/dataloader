import csv
from pathlib import Path

import numpy as np


def make_mulran_sequence(root, sequence="DCC01", stamps=None):
    source = Path(root) / sequence
    ouster = source / "sensor_data" / "Ouster"
    ouster.mkdir(parents=True)
    if stamps is None:
        stamps = [
            1_500_000_000_000_000_000,
            1_500_000_000_100_000_000,
        ]
    with (source / "sensor_data" / "data_stamp.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        for index, stamp in enumerate(stamps):
            points = np.array(
                [
                    [index + 1.0, 2.0, 3.0, 4.0],
                    [index + 5.0, 6.0, 7.0, 8.0],
                ],
                dtype="<f4",
            )
            points.tofile(str(ouster / "{}.bin".format(stamp)))
            writer.writerow([stamp, "ouster"])
    return source, stamps
