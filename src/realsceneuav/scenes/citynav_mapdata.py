"""CityNav map constants published by the official CityNav codebase.

Source:
https://github.com/water-cookie/citynav/blob/main/gsamllavanav/mapdata.py

Ground level is intentionally kept separate from the raster surface-height map:
the raster contains roofs/objects, while CityNav uses one reference ground level
per block when defining the aerial view footprint.
"""

GROUND_LEVEL_M: dict[str, float] = {
    "birmingham_block_0": 16.048856444156698,
    "birmingham_block_1": 10.02721669169363,
    "birmingham_block_2": 7.0204973115052995,
    "birmingham_block_3": 13.178169088193988,
    "birmingham_block_4": 12.510308110043573,
    "birmingham_block_5": 9.035724612581797,
    "birmingham_block_6": 8.342005752216762,
    "birmingham_block_7": 9.069972432079426,
    "birmingham_block_8": 9.767633576781428,
    "birmingham_block_9": 9.839007739084987,
    "birmingham_block_10": 8.163523742999931,
    "birmingham_block_11": 7.368852686277198,
    "birmingham_block_12": 4.4762829987220245,
    "birmingham_block_13": 7.38571378212217,
    "cambridge_block_2": 37.619963888870764,
    "cambridge_block_3": 37.80057158222191,
    "cambridge_block_4": 37.4447906884959,
    "cambridge_block_6": 35.84253359519473,
    "cambridge_block_7": 39.464565007249114,
    "cambridge_block_8": 41.62273423227555,
    "cambridge_block_9": 38.49224945510375,
    "cambridge_block_10": 34.02419401450328,
    "cambridge_block_12": 41.64058599525279,
    "cambridge_block_13": 43.69745461677479,
    "cambridge_block_14": 43.83640200944717,
    "cambridge_block_15": 37.940351251848824,
    "cambridge_block_16": 36.817109712105065,
    "cambridge_block_17": 35.684234558137426,
    "cambridge_block_18": 39.98595895231467,
    "cambridge_block_19": 41.064048426704154,
    "cambridge_block_20": 41.84850877729182,
    "cambridge_block_21": 36.74889242048286,
    "cambridge_block_22": 34.080272548771816,
    "cambridge_block_23": 34.436207572934606,
    "cambridge_block_25": 40.61091891460477,
    "cambridge_block_26": 42.35843426980253,
    "cambridge_block_27": 40.6977502452132,
    "cambridge_block_28": 31.608505851194376,
    "cambridge_block_32": 37.43283726180316,
    "cambridge_block_33": 38.33041905329609,
}


def citynav_ground_level(map_name: str) -> float:
    try:
        return GROUND_LEVEL_M[map_name]
    except KeyError as exc:
        raise KeyError(
            f"Unknown CityNav map {map_name!r}; pass ground_level_m explicitly."
        ) from exc
