#include "pinocchio/algorithm/geometry.hpp"

// Capsule struct used to simplify arguments to code generation
template<typename Scalar>
struct Capsule 
{
    // Ends coord. in reference frame
    Eigen::Matrix<Scalar, 3, 1> a;
    Eigen::Matrix<Scalar, 3, 1> b;
    // Radius
    Scalar radius;
    
    // Length
    Scalar getLength()
    {
        return (b-a).norm();
    }
    // Transform from parent to ref. frame
    pinocchio::SE3Tpl<Scalar> get_fMc()
    {   
        Scalar zero;
        zero = 0;
        Eigen::Matrix<Scalar, 3, 1> translation = 0.5*(a + b);
        Eigen::Matrix<Scalar, 3, 3> rotation;
        Eigen::Matrix<Scalar, 3, 1> capsDir = (b-a)/((b-a).norm());
        Eigen::Matrix<Scalar, 3, 1> upDir;
        upDir[0] = 0;
        upDir[1] = 0;
        upDir[2] = 1;
        Eigen::Matrix<Scalar, 3, 1> v;
        v = upDir.cross(capsDir);
        Scalar c = upDir.transpose()*capsDir;
        Scalar s = v.norm();

        // Check parallel case
        if(s > 1e-5)
        {
            Eigen::Matrix<Scalar, 3, 3> kmat;
            kmat << zero, -v[2], v[1],
                    v[2], zero, -v[0],
                    -v[1], v[0], zero;
            rotation = Eigen::Matrix<Scalar, 3, 3>::Identity(3,3) + kmat + ((1-c)/(s*s))*(kmat*kmat);            
        } else {
            rotation = Eigen::Matrix<Scalar, 3, 3>::Identity(3,3);
        }

        const pinocchio::SE3Tpl<Scalar> fMc(rotation, translation);

        return fMc;
    }

    template<typename AltScalar>
    Capsule<AltScalar> cast()
    {
        Eigen::Matrix<AltScalar, 3, 1> a_cast;
        a_cast[0] = a[0];
        a_cast[1] = a[1];
        a_cast[2] = a[2];
        Eigen::Matrix<AltScalar, 3, 1> b_cast;
        b_cast[0] = b[0];
        b_cast[1] = b[1];
        b_cast[2] = b[2];
        AltScalar radius_cast;
        radius_cast = radius;
        //pinocchio::SE3Tpl<AltScalar> fMc_cast(fMc.rotation(), fMc.translation());
        const struct Capsule<AltScalar> caps_cast = {a_cast, b_cast, radius_cast};

        return caps_cast;
    }
};

template<typename Scalar>
struct Sphere 
{
    // Center coord. in reference frame
    Eigen::Matrix<Scalar, 3, 1> c;
    // Radius
    Scalar radius;
    
    // Transform from parent to ref. frame
    pinocchio::SE3Tpl<Scalar> get_fMc()
    {   
        Scalar zero;
        zero = 0;
        Eigen::Matrix<Scalar, 3, 1> translation = c;
        Eigen::Matrix<Scalar, 3, 3> rotation;
        rotation = Eigen::Matrix<Scalar, 3, 3>::Identity(3,3);

        const pinocchio::SE3Tpl<Scalar> fMc(rotation, translation);

        return fMc;
    }

    template<typename AltScalar>
    Sphere<AltScalar> cast()
    {
        Eigen::Matrix<AltScalar, 3, 1> c_cast;
        c_cast[0] = c[0];
        c_cast[1] = c[1];
        c_cast[2] = c[2];
        AltScalar radius_cast;
        radius_cast = radius;
        //pinocchio::SE3Tpl<AltScalar> fMc_cast(fMc.rotation(), fMc.translation());
        const struct Sphere<AltScalar> sphere_cast = {c_cast, radius_cast};

        return sphere_cast;
    }
};

template<typename Scalar>
struct RectSweptSph 
{
    // Top-left corner coord. in reference (base link) frame
    Eigen::Matrix<Scalar, 3, 1> c;
    // length : x-dim
    // width : y-dim
    // radius of the swiwping sphere
    Scalar length;
    Scalar width;
    Scalar radius;
    
    // Transform from parent to ref. frame
    pinocchio::SE3Tpl<Scalar> get_fMc()
    {   
        Scalar zero;
        zero = 0;
        Eigen::Matrix<Scalar, 3, 1> translation = c;
        Eigen::Matrix<Scalar, 3, 3> rotation;
        rotation = Eigen::Matrix<Scalar, 3, 3>::Identity(3,3);

        const pinocchio::SE3Tpl<Scalar> fMc(rotation, translation);

        return fMc;
    }

    template<typename AltScalar>
    RectSweptSph<AltScalar> cast()
    {
        Eigen::Matrix<AltScalar, 3, 1> c_cast;
        c_cast[0] = c[0];
        c_cast[1] = c[1];
        c_cast[2] = c[2];
        AltScalar length_cast;
        AltScalar width_cast;
        AltScalar radius_cast;
        length_cast = length;
        width_cast = width;
        radius_cast = radius;
        const struct RectSweptSph<AltScalar> rss_cast = {c_cast, length_cast, width_cast, radius_cast};

        return rss_cast;
    }
};