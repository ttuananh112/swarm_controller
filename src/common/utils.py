from typing import Union

import numpy as np


def roll(matrix: np.ndarray, shift: int, axis: int, padding_value: Union[float, int]):
    rolled = np.roll(matrix, shift=shift, axis=axis)
    if shift < 0:
        rolled[:, shift:] = padding_value
    else:
        rolled[:, :shift] = padding_value
    return rolled
