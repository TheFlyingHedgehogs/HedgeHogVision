from .RealWorldTag import KnownTag

class MegaTag(KnownTag):
    def __init__(self, x: float, y: float, z: float, rotation_degrees: float, tag_witdth: float):
        KnownTag.__init__(self, 99, x, y, z, rotation_degrees)