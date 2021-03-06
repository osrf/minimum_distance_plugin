/*
 * Copyright (C) 2018, Matt Zucker
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Original source: https://github.com/mzucker/ccdwrapper
 */

#ifndef _CCD_EIGEN_H_
#define _CCD_EIGEN_H_

#include <ccd/ccd.h>
#include <Eigen/Geometry>
#include <iostream>
#include <type_traits>

static_assert(std::is_same<double, ccd_real_t>::value,
    "This plugin requires libccd to be built with double precision support.");

namespace ccdw {

typedef Eigen::Matrix<ccd_real_t, 3, 1> vec3;
typedef Eigen::Matrix<ccd_real_t, 4, 1> vec4;

typedef Eigen::Matrix<ccd_real_t, 3, 3> mat3;
typedef Eigen::Matrix<ccd_real_t, 4, 4> mat4;
typedef Eigen::Quaternion<ccd_real_t> quat;

class Transform3 {
public:

    Transform3():
        _rotation(quat::Identity()),
        _translation(0,0,0) { _update(); }

    explicit Transform3(const vec3& t):
        _rotation(quat::Identity()),
        _translation(t) { _update(); }

    explicit Transform3(const quat& r):
        _rotation(r),
        _translation(0, 0, 0) { _update(); }

    Transform3(const quat& q, const vec3& t):
        _rotation(q), _translation(t) { _update(); }

    Transform3 inverse() const {
        return Transform3(rotation().inverse(),
                          -(rotation().inverse()*translation()));

    }

    void setTranslation(const vec3& t) {
        _translation = t;
        _update();
    }

    void setRotation(const quat& q) {
        _rotation = q;
        _update();
    }

    const vec3& translation() const {
        return _translation;
    }

    const quat& rotation() const {
        return _rotation;
    }

    const Eigen::Affine3d& tobj() const {
        return _xform;
    }

    mat4 matrix() const {
        return _xform.matrix();
    }

    vec3 transformInv(const vec3& p) const {
        return _xform_inv * p;
    }

    vec3 transformFwd(const vec3& p) const {
        return _xform * p;
    }

    Transform3 compose(const Transform3& other) const {
        return Transform3(this->rotation() * other.rotation(),
                          this->translation() + this->rotation()*other.translation());
    }

private:

    quat _rotation;
    vec3 _translation;

    Eigen::Affine3d _xform;
    Eigen::Affine3d _xform_inv;

    void _update() {
        _xform = Eigen::Translation<double, 3>(_translation) * _rotation;
        _xform_inv = _rotation.inverse() * Eigen::Translation<double, 3>(-_translation);
    }

};

inline vec3 operator*(const Transform3& x, const vec3& v) {
    return x.transformFwd(v);
}

inline Transform3 operator*(const Transform3& x1, const Transform3& x2) {
    return x1.compose(x2);
}

class Box3 {
public:
    vec3 p0;
    vec3 p1;

    Box3(): p0(1,1,1), p1(0,0,0) {}
    Box3(const vec3& a, const vec3& b): p0(a), p1(b) {}

};

inline quat quatFromTwoVectors(const vec3& yy,
                               const vec3& z) {

    vec3 x = yy.cross(z);
    vec3 y = z.cross(x);

    mat3 m;
    m.col(0) = x / x.norm();
    m.col(1) = y / y.norm();
    m.col(2) = z / z.norm();

    return quat(m);

}

inline quat quatFromOneVector(const vec3& z) {

    int maxaxis = 0;
    if (fabs(z[1]) > fabs(z[maxaxis])) { maxaxis = 1; }
    if (fabs(z[2]) > fabs(z[maxaxis])) { maxaxis = 2; }

    vec3 y = vec3(0, 0, 0);
    y[(maxaxis + 1) % 3] = 1;

    return quatFromTwoVectors(y, z);

}

inline quat quatFromAxisAngle(const vec3& axis,
                              double angle) {

    return quat(Eigen::AngleAxisd(angle, axis));

}

inline quat quatFromOmega(const vec3& v) {
    double l = v.norm();
    vec3 vn = v / l;
    return quat(Eigen::AngleAxisd(l, vn));
}

} // namespace ccdw

#endif
