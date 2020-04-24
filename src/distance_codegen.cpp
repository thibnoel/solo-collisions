/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
 *
 *  CppADCodeGen is distributed under multiple licenses:
 *
 *   - Eclipse Public License Version 1.0 (EPL1), and
 *   - GNU General Public License Version 3 (GPL3).
 *
 *  EPL1 terms and conditions can be found in the file "epl-v10.txt", while
 *  terms and conditions for the GPL3 can be found in the file "gpl3.txt".
 * ----------------------------------------------------------------------------
 * Author: Joao Leal
 */
#include <iosfwd>
//#include <cppad/cg.hpp>
#include <pinocchio/codegen/cppadcg.hpp>

using namespace CppAD;
using namespace CppAD::cg;

template<typename Scalar>
Scalar computeDistanceFromPoints(Scalar x0,
                               Scalar y0, 
                               Scalar z0,
                               Scalar x1,
                               Scalar y1, 
                               Scalar z1){

    Scalar distanceResult = (x1 - x0)*(x1 - x0) + (y1 - y0)*(y1 - y0) + (z1 - z0)*(z1 - z0);

    return distanceResult;
}

template<typename Scalar>
Scalar computeDistanceFromSegments(Scalar x10,
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
    Scalar s_den;
    Scalar t_den;
    Scalar s_num; // = 0;
    Scalar t_num; // = 0;

    Scalar s_closest; // = 0;
    Scalar t_closest; // = 0;
    
    // Setting threshold on denominator for parallel case
    Scalar DEN_THRESHOLD;
    DEN_THRESHOLD = 1e-9;

    // For if tests
    Scalar out0_if;
    Scalar out1_if;
    Scalar condZero;
    condZero = 0.;

    /*------------- Computing the closest points and shortest distance -------------*/
    // Parallel case
    /*
    if (den < DEN_THRESHOLD) {
        s_num =  0 ;
        s_den = 1 ;
        t_num = vTw ;
        t_den = vTv ;
    }
    else {
        s_num = uTv*vTw - vTv*uTw ;
        s_den = den;
        t_num = uTu*vTw - uTv*uTw ;
        t_den = den;
    }*/
    // Conditional value of s_num (testing parallel case)
    out1_if = 0;
    out0_if = uTv*vTw - vTv*uTw ;
    s_num = CondExpLt(den, DEN_THRESHOLD, out1_if, out0_if);
    // Conditional value of s_den (testing parallel case)
    out1_if = 1;
    out0_if = den;
    s_den = CondExpLt(den, DEN_THRESHOLD, out1_if, out0_if);
    // Conditional value of t_num (testing parallel case)
    out1_if = vTw;
    out0_if = uTu*vTw - uTv*uTw ;
    t_num = CondExpLt(den, DEN_THRESHOLD, out1_if, out0_if);
    // Conditional value of t_den (testing parallel case)
    out1_if = vTv;
    out0_if = den;
    t_den = CondExpLt(den, DEN_THRESHOLD, out1_if, out0_if);

    // Check the constraint s in [0,1]
    /*
    if (s_num < 0) {
        s_num = 0 ;     // constrain s to 0 
        t_num = vTw ;
        t_den = vTv ;
    }
    else if (s_num > s_den) {
        s_num = s_den ;  // constrain s to 1
        t_num = vTw + uTv ;
        t_den = vTv ;
    }*/

    // Conditional value of s_num (testing constraint on s)
    out1_if = 0;
    out0_if = s_num;
    s_num = CondExpLt(s_num, condZero, out1_if, out0_if);
    out1_if = s_den;
    out0_if = s_num;
    s_num = CondExpGt(s_num, s_den, out1_if, out0_if);

    // Conditional value of t_num (testing constraint on s)
    out1_if = vTw;
    out0_if  = t_num;
    t_num = CondExpLt(s_num, condZero, out1_if, out0_if);
    out1_if = vTw + uTv;
    out0_if = t_num;
    t_num = CondExpGt(s_num, s_den, out1_if, out0_if);

    // Conditional value of t_den (testing constraint on s)
    out1_if = vTv;
    out0_if = t_den;
    t_den = CondExpLt(s_num, condZero, out1_if, out0_if);
    t_den = CondExpGt(s_num, s_den, out1_if, out0_if);



    // Check the constraint t in [0,1]
    // REIMPLEMENTATION WITH CondExp TODO
    /*if (t_num < 0){
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

        out1_if = 0;
        out0_if = s_num;
        s_num = CondExpLt(-uTw, condZero, out1_if, out0_if)
        out1_if = s_den;
        out0_if = -uTw;
        s_num = CondExpGt(-uTw, uTu, out1_if, out0_if);
        out1_if = s_den;
        out0_if = uTu;
        s_den = CondExpGt(-uTw, uTu, out1_if, out0_if);
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

        out1_if = 0;
        out0_if = s_num;
        s_num = CondExpLt(-uTw + uTv, condZero, out1_if, out0_if)
        out1_if = s_den;
        out0_if = -uTw + uTv;
        s_num = CondExpGt(-uTw + uTv, uTu, out1_if, out0_if);
        out1_if = s_den;
        out0_if = uTu;
        s_den = CondExpGt(-uTw + uTv, uTu, out1_if, out0_if);
    }*/

    out1_if = 0;
    out0_if = t_num;
    t_num = CondExpLt(t_num, condZero, out1_if, out0_if);
    out1_if = t_den;
    out0_if = t_num;
    t_num = CondExpGt(t_num, t_den, out1_if, out0_if);
    // Test against what should s be compared
    Scalar s_comp;
    s_comp = s_num;

    out1_if = -uTw;
    out0_if = s_num;
    s_comp = CondExpLt(t_num, condZero, out1_if, out0_if);
    out1_if = -uTw + uTv;
    out0_if = s_num;
    s_comp = CondExpGt(t_num, t_den, out1_if, out0_if);

    out1_if = 0;
    out0_if = s_num;
    s_num = CondExpLt(s_comp, condZero, out1_if, out0_if);
    out1_if = s_den;
    out0_if = s_comp;
    s_num = CondExpGt(s_comp, uTu, out1_if, out0_if);
    out1_if = s_den;
    out0_if = uTu;
    s_den = CondExpGt(s_comp, uTu, out1_if, out0_if);

    // Final computation of s_closest, t_closest
    /*
    if(abs(s_num) < DEN_THRESHOLD){
        s_closest = 0.;
    }
    else{
        s_closest = s_num/s_den;
    }*/
    out1_if = 0.;
    out0_if = s_num/s_den ;
    s_closest = CondExpLt(abs(s_num), DEN_THRESHOLD, out1_if, out0_if);

    /*
    if(abs(t_num) < DEN_THRESHOLD){
        t_closest = 0.;
    }
    else{
        t_closest = t_num/t_den;
    }*/
    out1_if = 0.;
    out0_if = t_num/t_den ;
    t_closest = CondExpLt(abs(t_num), DEN_THRESHOLD, out1_if, out0_if);


    //std::cout << s_closest << std::endl;
    //std::cout << t_closest << std::endl;

    Scalar diff_at_closest_X = w0 + s_closest*u0 - t_closest*v0;
    Scalar diff_at_closest_Y = w1 + s_closest*u1 - t_closest*v1;
    Scalar diff_at_closest_Z = w2 + s_closest*u2 - t_closest*v2;

    //std::cout << diff_at_closest_X << std::endl;
    //std::cout << diff_at_closest_Y << std::endl;
    //std::cout << diff_at_closest_Z << std::endl;
    
    Scalar distanceResult = sqrt(diff_at_closest_X*diff_at_closest_X + diff_at_closest_Y*diff_at_closest_Y + diff_at_closest_Z*diff_at_closest_Z);

    return distanceResult;
    }






int main(void) {
    // use a special object for source code generation
    //using CGD = CG<Scalar>;
    //using ADCG = AD<CGD>;

    typedef double Scalar;

    typedef CG<Scalar> CGScalar;
    typedef AD<CGScalar> ADScalar;
    typedef ADFun<CGScalar> ADFun;

    /***************************************************************************
     *                               the model
     **************************************************************************/

    CppAD::vector<ADScalar> x(12);
    /*x[0] = 4.;
    x[1] = 4.;
    x[2] = 4.;
    x[3] = 4.;
    x[4] = 4.;
    x[5] = 4.;
    x[6] = 4.;
    x[7] = 4.;
    x[8] = 4.;
    x[9] = 4.;
    x[10] = 4.;
    x[11] = 4.;*/
    Independent(x);

    CppAD::vector<ADScalar> y(1);
    y[0] = 1;

    //ADScalar a = computeDistanceFromPoints<ADScalar>(x[0],x[1],x[2],x[3],x[4],x[5]); 
    ADScalar a = computeDistanceFromSegments<ADScalar>(x[0],x[1],x[2],x[3],x[4],x[5],x[6],x[7],x[8],x[9],x[10],x[11]);

    y[0] = a;

    ADFun fun(x, y); // the model tape   

    /***************************************************************************
     *                        Generate the C source code
     **************************************************************************/

    /**
     * start the special steps for source code generation for a Jacobian
     */
    CodeHandler<Scalar> handler;

    CppAD::vector<CGScalar> indVars(12); // size of x (Independent variable)
    handler.makeVariables(indVars);

    CppAD::vector<CGScalar> gen_fun = fun.Forward(0, indVars);
    CppAD::vector<CGScalar> gen_jac = fun.SparseJacobian(indVars);

    // ??? Set parameters for code generation?
    LanguageC<Scalar> langC("double");
    LangCDefaultVariableNameGenerator<Scalar> nameGen;

    // Generate function code
    std::ostringstream code;
    handler.generateCode(code, langC, gen_fun, nameGen);

    // Generate jacobian code
    std::ostringstream code_jac;
    handler.generateCode(code_jac, langC, gen_jac, nameGen);
    
    std::cout << "// Generated dist(x) :\n";
    std::cout << code.str();
    std::cout << "// Generated dist'(x) :\n";
    std::cout << code_jac.str();

    /* Do a computation with the generated code
    CppAD::vector<ADScalar> test_x(12);
    test_x[0] = 0.;
    test_x[1] = 0.;
    test_x[2] = 0.;
    test_x[3] = 1.;
    test_x[4] = 1.;
    test_x[5] = 1.;
    test_x[6] = 0.;
    test_x[7] = 0.;
    test_x[8] = 2.;
    test_x[9] = 0.;
    test_x[10] = 0.;
    test_x[11] = 3.;

    CppAD::vector<ADScalar> test_y(1);*/

}