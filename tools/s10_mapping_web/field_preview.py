"""Bounded offline PCD display decoder; never changes the vendor map."""
import math
import struct


def read_pcd_preview(path, limit=30000):
    if not 1 <= limit <= 60000:
        raise ValueError('Invalid display point budget')
    with open(path, 'rb') as f:
        header = {}
        for _ in range(100):
            line = f.readline(4097)
            if not line or len(line) > 4096:
                raise ValueError('Invalid PCD header')
            parts = line.decode('ascii').strip().split()
            if not parts or parts[0].startswith('#'):
                continue
            header[parts[0]] = parts[1:]
            if parts[0] == 'DATA':
                break
        else:
            raise ValueError('PCD header exceeds bound')
        fields = header['FIELDS']
        sizes, counts = list(map(int, header['SIZE'])), list(map(int, header.get('COUNT', ['1']*len(fields))))
        types = header['TYPE']
        if not len(fields) == len(sizes) == len(counts) == len(types) or len(set(fields)) != len(fields):
            raise ValueError('PCD field declaration mismatch')
        if any(s not in (1, 2, 4, 8) or c < 1 or c > 16 for s, c in zip(sizes, counts)):
            raise ValueError('Unsupported PCD field size/count')
        count = int(header['POINTS'][0])
        if not 1 <= count <= 10000000:
            raise ValueError('PCD point count outside display budget')
        indices = [fields.index(k) for k in ('x', 'y', 'z')]
        if any(types[i] != 'F' or sizes[i] not in (4, 8) or counts[i] != 1 for i in indices):
            raise ValueError('PCD requires scalar float XYZ')
        step, offsets = sum(s*c for s, c in zip(sizes, counts)), []
        for i in indices:
            offsets.append(sum(s*c for s, c in zip(sizes[:i], counts[:i])))
        if step > 1024:
            raise ValueError('PCD row too large')
        stride = math.ceil(count/limit)
        points, data_start = [], f.tell()
        mode = header['DATA'][0]
        if mode == 'binary':
            f.seek(0, 2)
            if f.tell()-data_start != count*step:
                raise ValueError('PCD binary size mismatch')
            readers = [struct.Struct('<f' if sizes[i] == 4 else '<d') for i in indices]
            for n in range(0, count, stride):
                f.seek(data_start+n*step)
                raw = f.read(step)
                point = [r.unpack_from(raw, o)[0] for r, o in zip(readers, offsets)]
                if all(math.isfinite(v) for v in point):
                    points.append([round(v, 3) for v in point])
        elif mode == 'ascii':
            ai = [sum(counts[:i]) for i in indices]
            for n in range(count):
                line = f.readline(16385)
                if not line or len(line) > 16384:
                    raise ValueError('PCD ASCII truncated or oversized')
                if n % stride == 0:
                    row = line.split()
                    point = [float(row[i]) for i in ai]
                    if all(math.isfinite(v) for v in point):
                        points.append([round(v, 3) for v in point])
        else:
            raise ValueError('此 PCD 编码尚未准备离线显示资源；不伪造叠图')
        if not points:
            raise ValueError('PCD has no finite display points')
        return points
