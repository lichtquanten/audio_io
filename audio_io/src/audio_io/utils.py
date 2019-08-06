def switch_endianness(data, width):
    """
    Parameters
    ----------
    data : str
        Raw data.
    width : int
        Number of bytes per sample.

    Returns
    -------
    str
    """
    out = [None] * len(data)
    idx = 0
    for i in xrange(0, len(data), width):
        for j in xrange(width, 0, -1):
            out[idx] = data[i + j - 1]
            idx += 1
    return str(bytearray(out))

def width_to_dtype(width):
    """
    Parameters
    ----------
    width : int
        Number of bytes per sample.

    Returns
    -------
    np.dtype
    """
    if width == 1:
        return np.int8
    if width == 2:
        return np.int16
    if width == 4:
        return np.int32
    if width == 8:
        return np.int64
    return None
