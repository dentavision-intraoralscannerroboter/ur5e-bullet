import os
import tempfile

import ur5e_bullet

fst = ur5e_bullet


def test_frustum_dims():
    d = fst._frustum_dims(far=0.03, near=0.001, fov_deg=87,
                          sensor_w_mm=36.0, sensor_h_mm=20.25)
    assert abs(d["half_w"] - 0.03 * 0.949 - 0.0015) < 1e-3
    assert abs(d["half_h"] - 0.03 * 0.5338) < 2e-3
    assert abs(d["n_ratio"] - 0.001 / 0.03) < 1e-9
    assert d["far"] == 0.03 and d["near"] == 0.001


def test_build_frustum_obj():
    with tempfile.TemporaryDirectory() as root:
        p = os.path.join(root, "frustum.obj")
        assert fst._build_frustum_obj(p, n_ratio=0.1) is True
        assert fst._build_frustum_obj(p, n_ratio=0.1) is False  # existiert bereits
        verts, faces = [], []
        with open(p) as fh:
            for line in fh:
                parts = line.split()
                if parts and parts[0] == "v":
                    verts.append(tuple(map(float, parts[1:])))
                elif parts and parts[0] == "f":
                    faces.append(parts[1:])
        assert len(verts) == 8
        assert len(faces) == 12
        assert all(len(f) == 3 for f in faces)


def test_frustum_winding_outward():
    # Alle Dreiecksnormalen zeigen vom Volumen-Zentrum nach aussen.
    def read(path):
        verts, faces = [], []
        with open(path) as fh:
            for line in fh:
                parts = line.split()
                if parts and parts[0] == "v":
                    verts.append(tuple(map(float, parts[1:])))
                elif parts and parts[0] == "f":
                    faces.append([int(i) - 1 for i in parts[1:]])
        return verts, faces

    with tempfile.TemporaryDirectory() as root:
        p = os.path.join(root, "frustum.obj")
        fst._build_frustum_obj(p, n_ratio=0.1)
        verts, faces = read(p)
        center = [sum(v[k] for v in verts) / len(verts) for k in range(3)]
        for tri in faces:
            p0, p1, p2 = (verts[i] for i in tri)
            e1 = [p1[k] - p0[k] for k in range(3)]
            e2 = [p2[k] - p0[k] for k in range(3)]
            n = [e1[1] * e2[2] - e1[2] * e2[1],
                 e1[2] * e2[0] - e1[0] * e2[2],
                 e1[0] * e2[1] - e1[1] * e2[0]]
            c = [sum(v[k] for v in (p0, p1, p2)) / 3 for k in range(3)]
            dot = sum(n[k] * (c[k] - center[k]) for k in range(3))
            assert dot > 0, f"Flaeche {tri} zeigt nach innen"


if __name__ == "__main__":
    fn = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for f in fn:
        f()
        print("OK", f.__name__)
    print("ALLE FRUSTUM-TESTS OK")