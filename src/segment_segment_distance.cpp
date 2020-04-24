// Computes the distance between 2 segments, 
// given each as pairs of 3D points
#include <array>
#include <iostream>
#include <eigen3/Eigen/Core>

class Segment {       // Segment class
  public:             // Access specifier
    Eigen::Matrix<double,3,1> p0;  
    Eigen::Matrix<double,3,1> p1;  // Extremities as 3D eigen vectors

    
    Segment(Eigen::Matrix<double,3,1> p0, Eigen::Matrix<double,3,1> p1){
        this->p0 = p0;
        this->p1 = p1;
    }
};

template<typename Scalar>
/*
The segments are passed as 3D vectors, one for each endpoint
*/
void computeDistance(Segment seg1, Segment seg2, Scalar& distanceResult)
{   
    /*------------- Setting up the variables -------------*/   
    //Scalar distanceResult = INFINITY;

    // Direction vector of seg1
    Eigen::Matrix<Scalar,3,1> u = seg1.p1 - seg1.p0;
    // Direction vector of seg2 
    Eigen::Matrix<Scalar,3,1> v = seg2.p1 - seg2.p0;
    // Direction between segments origins
    Eigen::Matrix<Scalar,3,1> w = seg1.p0 - seg2.p0;

    // Vector products
    Scalar uTu = u.transpose()*u;
    Scalar uTv = u.transpose()*v;
    Scalar vTv = v.transpose()*v;
    Scalar uTw = u.transpose()*w;
    Scalar vTw = v.transpose()*w;

    // Solving denominators (den) and numerators (num)
    Scalar den = uTu*vTv - uTv*uTv;
    Scalar s_den = den;
    Scalar t_den = den;
    Scalar s_num = 0;
    Scalar t_num = 0;

    Scalar s_closest = 0;
    Scalar t_closest = 0;
    
    // Setting threshold on denominator for parallel case
    Scalar DEN_THRESHOLD = 1e-9;

    /*------------- Computing the closest points and shortest distance -------------*/
    // Parallel case
    if (den < DEN_THRESHOLD) {
        s_num =  0 ;
        s_den = 1 ;
        t_num = vTw ;
        t_den = vTv ;
    }
    else {
        s_num = uTv*vTw - vTv*uTw ;
        t_num = uTu*vTw - uTv*uTw ;
    }

    // Check the constraint s in [0,1]
    if (s_num < 0) {
        s_num = 0 ;     // constrain s to 0 
        t_num = vTw ;
        t_den = vTv ;
    }
    else if (s_num > s_den) {
        s_num = s_den ;  // constrain s to 1
        t_num = vTw + uTv ;
        t_den = vTv ;
    }

    // Check the constraint t in [0,1]
    if (t_num < 0){
        t_num = 0 ;  // constrain t to 0 
        // Re check constrain on s
        if (-uTw < 0){
            s_num = 0 ;
        }
        else if (-uTw > uTu) {
            s_num = s_den ;
        }
        else {
            s_num = -uTw ;
            s_den = uTu ;
        }
    }
    else if (t_num > t_den) {
        t_num = t_den ;  // constrain t to 1
        if (-uTw + uTv < 0) {
            s_num = 0 ;
        }
        else if ((-uTw + uTv) > uTu){
            s_num = s_den ;
        }
        else {
            s_num = -uTw + uTv ;
            s_den = uTu ;
        }
    }

    s_closest = (std::abs(s_num) < DEN_THRESHOLD ? 0.0 : s_num / s_den);
    t_closest = (std::abs(t_num) < DEN_THRESHOLD ? 0.0 : t_num / t_den);

    Eigen::Matrix<Scalar,3,1> diff_at_closest = w + s_closest*u - t_closest*v;

    std::cout << diff_at_closest[0] << std::endl;
    std::cout << diff_at_closest[1] << std::endl;
    std::cout << diff_at_closest[2] << std::endl;

    distanceResult = diff_at_closest.norm();
    //return distanceResult;
}

template<typename Scalar>
Scalar computeDistanceFromPoints(Scalar x10,
                               Scalar y10, 
                               Scalar z10,
                               Scalar x11,
                               Scalar y11, 
                               Scalar z11,
                               Scalar x20,
                               Scalar y20,
                               Scalar z20,
                               Scalar x21,
                               Scalar y21,
                               Scalar z21){
    //Scalar distanceResult = INFINITY;

    Scalar u0 = x11 - x10;
    Scalar u1 = y11 - y10;
    Scalar u2 = z11 - z10;
    Scalar v0 = x21 - x20;
    Scalar v1 = y21 - y20;
    Scalar v2 = z21 - z20;
    Scalar w0 = x10 - x20;
    Scalar w1 = y10 - y20;
    Scalar w2 = z10 - z20;

    Scalar uTu = (u0*u0 + u1*u1 + u2*u2); 
    Scalar uTv = (u0*v0 + u1*v1 + u2*v2);
    Scalar vTv = (v0*v0 + v1*v1 + v2*v2);
    Scalar uTw = (u0*w0 + u1*w1 + u2*w2);
    Scalar vTw = (v0*w0 + v1*w1 + v2*w2);

    // Solving denominators (den) and numerators (num)
    Scalar den = uTu*vTv - uTv*uTv;
    Scalar s_den = den;
    Scalar t_den = den;
    Scalar s_num = 0;
    Scalar t_num = 0;

    Scalar s_closest = 0;
    Scalar t_closest = 0;
    
    // Setting threshold on denominator for parallel case
    Scalar DEN_THRESHOLD = 1e-9;

    /*------------- Computing the closest points and shortest distance -------------*/
    // Parallel case
    if (den < DEN_THRESHOLD) {
        s_num =  0 ;
        s_den = 1 ;
        t_num = vTw ;
        t_den = vTv ;
    }
    else {
        s_num = uTv*vTw - vTv*uTw ;
        t_num = uTu*vTw - uTv*uTw ;
    }

    // Check the constraint s in [0,1]
    if (s_num < 0) {
        s_num = 0 ;     // constrain s to 0 
        t_num = vTw ;
        t_den = vTv ;
    }
    else if (s_num > s_den) {
        s_num = s_den ;  // constrain s to 1
        t_num = vTw + uTv ;
        t_den = vTv ;
    }

    // Check the constraint t in [0,1]
    if (t_num < 0){
        t_num = 0 ;  // constrain t to 0 
        // Re check constrain on s
        if (-uTw < 0){
            s_num = 0 ;
        }
        else if (-uTw > uTu) {
            s_num = s_den ;
        }
        else {
            s_num = -uTw ;
            s_den = uTu ;
        }
    }
    else if (t_num > t_den) {
        t_num = t_den ;  // constrain t to 1
        if (-uTw + uTv < 0) {
            s_num = 0 ;
        }
        else if ((-uTw + uTv) > uTu){
            s_num = s_den ;
        }
        else {
            s_num = -uTw + uTv ;
            s_den = uTu ;
        }
    }

    s_closest = (abs(s_num) < DEN_THRESHOLD ? 0.0 : s_num / s_den);
    t_closest = (abs(t_num) < DEN_THRESHOLD ? 0.0 : t_num / t_den);

    std::cout << s_closest << std::endl;
    std::cout << t_closest << std::endl;

    //Eigen::Matrix<Scalar,3,1> diff_at_closest = w + s_closest*u - t_closest*v;
    Scalar diff_at_closest_X = w0 + s_closest*u0 - t_closest*v0;
    Scalar diff_at_closest_Y = w1 + s_closest*u1 - t_closest*v1;
    Scalar diff_at_closest_Z = w2 + s_closest*u2 - t_closest*v2;

    std::cout << diff_at_closest_X << std::endl;
    std::cout << diff_at_closest_Y << std::endl;
    std::cout << diff_at_closest_Z << std::endl;
    
    Scalar distanceResult = std::sqrt(diff_at_closest_X*diff_at_closest_X + diff_at_closest_Y*diff_at_closest_Y + diff_at_closest_Z*diff_at_closest_Z);

    return distanceResult;
    }
/*
int main(void) {
    typedef double Scalar;

    Eigen::Vector3d p10(0,0,0);
    Eigen::Vector3d p11(1,1,1);
    Eigen::Vector3d p20(0,0,2);
    Eigen::Vector3d p21(0,0,3);

    Segment s1(p10,p11);
    Segment s2(p20,p21);

    double distResult = INFINITY;
    //Scalar distResultP = INFINITY;
    //Scalar cDist = computeDistance(s1, s2);
    computeDistance(s1,s2, distResult);
    Scalar distResultP = computeDistanceFromPoints<double>(0,0,0,1,1,1,0,0,2,0,0,3);

    std::cout << "Shortest distance (vectors) :\n" << distResult << std::endl;
    std::cout << "Shortest distance (Scalars) :\n" << distResultP << std::endl;
}*/