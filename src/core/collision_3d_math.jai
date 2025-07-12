#import "Basic";
#import "Math";

Vector3 :: struct {
    x, y, z: float;
}

Quaternion :: struct {
    x, y, z, w: float;
}

Matrix4 :: struct {
    m: [4][4] float;
}

Transform :: struct {
    position: Vector3;
    rotation: Quaternion;
    scale: Vector3;
}

EPSILON :: 0.00001;

operator + :: (a: Vector3, b: Vector3) -> Vector3 {
    return .{a.x + b.x, a.y + b.y, a.z + b.z};
}

operator - :: (a: Vector3, b: Vector3) -> Vector3 {
    return .{a.x - b.x, a.y - b.y, a.z - b.z};
}

operator * :: (a: Vector3, s: float) -> Vector3 {
    return .{a.x * s, a.y * s, a.z * s};
}

operator * :: (s: float, a: Vector3) -> Vector3 {
    return .{a.x * s, a.y * s, a.z * s};
}

dot :: (a: Vector3, b: Vector3) -> float {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

cross :: (a: Vector3, b: Vector3) -> Vector3 {
    return .{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

length :: (v: Vector3) -> float {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

length_squared :: (v: Vector3) -> float {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

normalize :: (v: Vector3) -> Vector3 {
    len := length(v);
    if len < EPSILON return .{0, 0, 0};
    return v * (1.0 / len);
}

quaternion_from_axis_angle :: (axis: Vector3, angle: float) -> Quaternion {
    half_angle := angle * 0.5;
    s := sin(half_angle);
    return .{
        axis.x * s,
        axis.y * s,
        axis.z * s,
        cos(half_angle)
    };
}

quaternion_multiply :: (a: Quaternion, b: Quaternion) -> Quaternion {
    return .{
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y + a.y * b.w + a.z * b.x - a.x * b.z,
        a.w * b.z + a.z * b.w + a.x * b.y - a.y * b.x,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    };
}

quaternion_rotate_vector :: (q: Quaternion, v: Vector3) -> Vector3 {
    qv := Vector3.{q.x, q.y, q.z};
    uv := cross(qv, v);
    uuv := cross(qv, uv);
    return v + ((uv * q.w) + uuv) * 2.0;
}

matrix4_from_transform :: (t: Transform) -> Matrix4 {
    m: Matrix4;
    
    xx := t.rotation.x * t.rotation.x;
    yy := t.rotation.y * t.rotation.y;
    zz := t.rotation.z * t.rotation.z;
    xy := t.rotation.x * t.rotation.y;
    xz := t.rotation.x * t.rotation.z;
    yz := t.rotation.y * t.rotation.z;
    wx := t.rotation.w * t.rotation.x;
    wy := t.rotation.w * t.rotation.y;
    wz := t.rotation.w * t.rotation.z;
    
    m.m[0][0] = (1.0 - 2.0 * (yy + zz)) * t.scale.x;
    m.m[0][1] = 2.0 * (xy + wz) * t.scale.x;
    m.m[0][2] = 2.0 * (xz - wy) * t.scale.x;
    m.m[0][3] = 0.0;
    
    m.m[1][0] = 2.0 * (xy - wz) * t.scale.y;
    m.m[1][1] = (1.0 - 2.0 * (xx + zz)) * t.scale.y;
    m.m[1][2] = 2.0 * (yz + wx) * t.scale.y;
    m.m[1][3] = 0.0;
    
    m.m[2][0] = 2.0 * (xz + wy) * t.scale.z;
    m.m[2][1] = 2.0 * (yz - wx) * t.scale.z;
    m.m[2][2] = (1.0 - 2.0 * (xx + yy)) * t.scale.z;
    m.m[2][3] = 0.0;
    
    m.m[3][0] = t.position.x;
    m.m[3][1] = t.position.y;
    m.m[3][2] = t.position.z;
    m.m[3][3] = 1.0;
    
    return m;
}

transform_point :: (m: Matrix4, p: Vector3) -> Vector3 {
    return .{
        m.m[0][0] * p.x + m.m[1][0] * p.y + m.m[2][0] * p.z + m.m[3][0],
        m.m[0][1] * p.x + m.m[1][1] * p.y + m.m[2][1] * p.z + m.m[3][1],
        m.m[0][2] * p.x + m.m[1][2] * p.y + m.m[2][2] * p.z + m.m[3][2]
    };
}

transform_direction :: (m: Matrix4, d: Vector3) -> Vector3 {
    return .{
        m.m[0][0] * d.x + m.m[1][0] * d.y + m.m[2][0] * d.z,
        m.m[0][1] * d.x + m.m[1][1] * d.y + m.m[2][1] * d.z,
        m.m[0][2] * d.x + m.m[1][2] * d.y + m.m[2][2] * d.z
    };
}