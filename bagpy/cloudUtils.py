from cloudpathlib import GSPath

def open_cloud_save(path, *args, **kwargs):
    if type(path) == GSPath:
        return path.open(*args, **kwargs)
    return open(*args, **kwargs)