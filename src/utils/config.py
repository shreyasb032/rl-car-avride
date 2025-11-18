import yaml

class Config:
    def __init__(self, path="configs/default.yaml"):
        with open(path, "r") as f:
            self.cfg = yaml.safe_load(f)

    def __getitem__(self, key):
        return self.cfg.get(key)

    def get(self, *keys, default=None):
        # Optional helper for nested lookup
        node = self.cfg
        for k in keys:
            node = node.get(k, {})
        return node or default
