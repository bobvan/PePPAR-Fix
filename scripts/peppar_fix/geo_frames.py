"""Coordinate reference frames — Frame, GeoPoint, and conversion.

The canonical processing frame is **ITRF2020 at the observation epoch**
— the frame the satellite orbits live in.  Every coordinate that
crosses a module or file boundary carries an explicit ``Frame``; bare
``(X, Y, Z)`` tuples are banned at boundaries (fine as locals inside a
single numeric routine).  Design: ``docs/coordinate-reference-frames.md``.

Conversion is **pyproj / PROJ**-backed (PROJ >= 9.0 for ITRF2020).
Validated against the OPUS UFO1 NAD83(2011)/ITRF2020 ground-truth pair:
the NAD83(2011) -> ITRF2020 14-param time-dependent Helmert reproduces
OPUS to ~5 cm and round-trips are self-consistent to << 1 mm.

Why ~5 cm and not < 1 cm: this is *not* transform-math error.  PROJ
ships HTDP-derived Helmert parameters via the EPSG database; OPUS
realizes the same transform through NGS's NCAT/HTDP.  The two are
different numerical realizations of the NAD83(2011)<->ITRF2020 tie and
differ by a few cm, on top of OPUS's own ~cm survey sigma.  The residual
is dominated by realization-vs-realization noise, not by this module —
and it is ~35x smaller than the 1.71 m (1710 mm) datum error this module
exists to kill.  Surveyed ARPs should therefore store the OPUS ITRF2020
value *directly* (exact); this transform is for coordinates not natively
in ITRF — NAD83 CORS/RTK fixes, a NAD83 ``--known-pos`` paste.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache

CANONICAL_REALIZATION = "ITRF2020"

# Geocentric (ECEF X/Y/Z, metres) CRS EPSG codes per realization.
#   ITRF2020 geocentric    : 9988
#   NAD83(2011) geocentric : 6317
#   ETRS89 geocentric      : 4936  (ensemble; ~cm ambiguous across ETRFxx)
#
# NOTE: "WGS84" is deliberately NOT a realization here.  GPS broadcast /
# NAV2 coordinates are nominally WGS84, but the *generic* WGS84 datum is
# an ensemble (~2 m ambiguous, EPSG:4978) that transforms badly.  The
# current WGS84 realization (G2296...) agrees with ITRF2020 to ~cm, so
# such coordinates are tagged `ITRF2020` directly — the cm-level WGS84↔
# ITRF difference is far below the 1.71 m datum error this module exists
# to kill, and well below current lab needs.
_REALIZATION_EPSG = {
    "ITRF2020": 9988,
    "NAD83(2011)": 6317,
    "ETRS89": 4936,
}

# Plate-fixed realizations: their epoch is a *reference convention*, not
# the observation time.  NAD83(2011) coords are expressed at 2010.0;
# ETRS89 coords are plate-fixed to stable Eurasia and coincide with ITRF
# at 1989.0 (EPSG:4936→ITRF2020 grows ~2.5 cm/yr from there — verified
# ~0.85 m at 2026.5).  A plate-fixed coord is time-invariant, so the
# dynamic frame (ITRF2020, WGS84) carries the real observation epoch;
# PROJ's time-dependent Helmert wants that observation epoch, which we
# therefore always take from the dynamic side (see convert()).
_STATIC_REALIZATIONS = {"NAD83(2011)", "ETRS89"}

# The reference epoch each static realization's coordinates are conventionally
# expressed at.  It labels the source Frame honestly; the numerical transform
# to/from a dynamic frame uses the *observation* (dynamic-side) epoch, so this
# is not what drives plate-motion propagation — that is obs_epoch in convert().
_STATIC_REFERENCE_EPOCH = {
    "NAD83(2011)": 2010.0,
    "ETRS89": 1989.0,
}


def static_reference_epoch(realization: str, default: float = 2010.0) -> float:
    """Conventional reference epoch for a plate-fixed realization (the epoch
    its coordinates are expressed at).  Used to tag a source Frame honestly;
    the transform itself uses the observation epoch from the dynamic side."""
    return _STATIC_REFERENCE_EPOCH.get(realization, default)


@dataclass(frozen=True)
class Frame:
    """A reference frame = (realization, epoch).

    ``realization`` names the datum realization ("ITRF2020",
    "NAD83(2011)", "WGS84"); ``epoch`` is the decimal year the
    coordinates are valid at (e.g. 2026.42).  Epoch matters because
    ITRF is dynamic (~2 cm/yr plate motion in North America).
    """

    realization: str
    epoch: float

    def __post_init__(self):
        if self.realization not in _REALIZATION_EPSG:
            raise ValueError(
                f"unknown realization {self.realization!r}; known: "
                f"{sorted(_REALIZATION_EPSG)}")

    @classmethod
    def parse(cls, s: str) -> "Frame":
        """Parse ``"ITRF2020@2026.42"`` -> Frame."""
        realization, sep, epoch = s.partition("@")
        if not sep or not epoch:
            raise ValueError(
                f"frame must be 'REALIZATION@EPOCH', got {s!r}")
        return cls(realization.strip(), float(epoch))

    def __str__(self) -> str:
        return f"{self.realization}@{self.epoch:.4f}"


@dataclass(frozen=True)
class GeoPoint:
    """An ECEF coordinate that carries its frame.  The only type that
    crosses a module/file boundary — never a bare tuple."""

    ecef: tuple  # (x, y, z) metres
    frame: Frame

    def __post_init__(self):
        if len(self.ecef) != 3:
            raise ValueError(f"ecef must be a 3-tuple, got {self.ecef!r}")
        if not isinstance(self.frame, Frame):
            raise TypeError(f"frame must be a Frame, got {type(self.frame)}")


@lru_cache(maxsize=None)
def _transformer(src_realization: str, dst_realization: str):
    import pyproj
    return pyproj.Transformer.from_crs(
        f"EPSG:{_REALIZATION_EPSG[src_realization]}",
        f"EPSG:{_REALIZATION_EPSG[dst_realization]}",
        always_xy=True,
    )


def convert(point: GeoPoint, target: Frame) -> GeoPoint:
    """Convert ``point`` to ``target`` frame (datum and/or epoch).

    Cross-realization (e.g. NAD83(2011) -> ITRF2020) goes through PROJ's
    time-dependent Helmert; the observation/output epoch is ``target
    .epoch`` (validated: that's the epoch PROJ propagates to).  Within
    one realization, an epoch change is plate-motion propagation; an
    equal-epoch convert is the identity.
    """
    if point.frame == target:
        return point
    if point.frame.realization == target.realization:
        if abs(point.frame.epoch - target.epoch) < 1e-9:
            return GeoPoint(point.ecef, target)
        # Same datum, different epoch = intra-realization plate motion.
        return _propagate_epoch(point, target)
    # Cross-realization datum transform.  t = the observation epoch =
    # the dynamic-side frame's epoch (NAD83(2011)'s 2010.0 is a
    # reference convention, not an obs time).  Taking t from the
    # dynamic side makes the transform its own inverse (round-trips to
    # << 1 mm).
    if target.realization not in _STATIC_REALIZATIONS:
        obs_epoch = target.epoch
    elif point.frame.realization not in _STATIC_REALIZATIONS:
        obs_epoch = point.frame.epoch
    else:
        obs_epoch = target.epoch  # static<->static: no time term
    tf = _transformer(point.frame.realization, target.realization)
    x, y, z = point.ecef
    nx, ny, nz, _t = tf.transform(x, y, z, obs_epoch)
    return GeoPoint((nx, ny, nz), target)


# Plate-fixed anchor used to propagate a dynamic frame across epochs:
# round-tripping ITRF2020@t1 -> NAD83(2011) -> ITRF2020@t2 applies the
# plate motion between t1 and t2 (because NAD83(2011) is plate-fixed,
# so the same physical point keeps one NAD83 coordinate while its ITRF
# coordinate moves with the plate).
_PLATE_ANCHOR = Frame("NAD83(2011)", 2010.0)


def _propagate_epoch(point: GeoPoint, target: Frame) -> GeoPoint:
    """Intra-realization epoch change (plate motion) for a dynamic
    frame, via the plate-fixed NAD83(2011) anchor.  ~2 cm/yr in CONUS."""
    if point.frame.realization in _STATIC_REALIZATIONS:
        # Static datum has no epoch-dependent motion — relabel only.
        return GeoPoint(point.ecef, target)
    anchored = convert(point, _PLATE_ANCHOR)
    return convert(anchored, target)


def to_canonical(point: GeoPoint, obs_epoch: float) -> GeoPoint:
    """Convert ``point`` to the canonical processing frame:
    ITRF2020 at ``obs_epoch`` (the satellite-orbit frame)."""
    return convert(point, Frame(CANONICAL_REALIZATION, obs_epoch))
