from pathlib import Path
import tempfile
import unittest
import numpy as np
from PIL import Image
from mapnav.png import write_rgb_png


class PngTests(unittest.TestCase):
    def test_lossless_padded_rows(self):
        rng = np.random.default_rng(9)
        image = rng.integers(0, 256, (27, 31, 3), dtype=np.uint8)
        pixels = b"".join(row.tobytes() + b"PAD!!" for row in image)
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "frame.png"
            write_rgb_png(path, 31, 27, 31 * 3 + 5, pixels)
            np.testing.assert_array_equal(np.array(Image.open(path)), image)
            with self.assertRaises(FileExistsError): write_rgb_png(path, 31, 27, 98, pixels)

    def test_invalid_layout_rejected(self):
        for width, height, step, pixels in [(0, 2, 6, b""), (2, 2, 3, b"123456"), (2, 2, 6, b"123456")]:
            with self.assertRaises(ValueError): write_rgb_png("unused.png", width, height, step, pixels)
