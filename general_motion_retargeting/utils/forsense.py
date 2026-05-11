from general_motion_retargeting.utils.forsense_vendor.bvh_loader import (
    MIXAMO_TO_GMR,
    SNAKE_TO_GMR,
    load_forsense_bvh,
)


# Backwards-compatible names used by docs and older imports.
FORSENSE_SNAKE_ALIASES = SNAKE_TO_GMR
FORSENSE_MIXAMO_ALIASES = MIXAMO_TO_GMR


def load_bvh_file(bvh_file, format="forsense"):
    if format != "forsense":
        raise ValueError(f"Invalid format for forsense loader: {format}")
    return load_forsense_bvh(bvh_file)
