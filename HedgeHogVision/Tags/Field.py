from .Tags import KnownTag

class Field:
    def __init__(self, *tags: KnownTag):
        self.tags = {}
        for i in tags:
            self.tags[i.id] = i

    def __getitem__(self, idx):
        if idx in self.tags.keys(): return self.tags[idx]
        return None
