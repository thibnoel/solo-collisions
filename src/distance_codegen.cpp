#include <iosfwd>
#include <pinocchio/codegen/cppadcg.hpp>
//not used
//#include "pinocchio/utils/static-if.hpp"
#include "segment_segment_distance.cpp"

using namespace CppAD;
using namespace CppAD::cg;

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

    Scalar u0 = x11 - x10; // u : direction vector of 1st segment
    Scalar u1 = y11 - y10;
    Scalar u2 = z11 - z10;
    Scalar v0 = x21 - x20; // v : direction vector of 2nd segment
    Scalar v1 = y21 - y20;
    Scalar v2 = z21 - z20;
    Scalar w0 = x10 - x20; // w : direction between first endpoints of both segments
    Scalar w1 = y10 - y20;
    Scalar w2 = z10 - z20;

    // Scalar products
    Scalar uTu = (u0*u0 + u1*u1 + u2*u2); 
    Scalar uTv = (u0*v0 + u1*v1 + u2*v2);
    Scalar vTv = (v0*v0 + v1*v1 + v2*v2);
    Scalar uTw = (u0*w0 + u1*w1 + u2*w2);
    Scalar vTw = (v0*w0 + v1*w1 + v2*w2);

    // Solving for s (resp. t) on segment 1 (resp. 2)
    // Initializing denominators (den) and numerators (num)
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

    // For condExp tests
    Scalar scalarZero;
    Scalar scalarOne;
    scalarZero = 0.;
    scalarOne = 1;

    /*------------- Computing the closest points and shortest distance -------------*/
    // Parallel case
    // Conditional value of s_num (testing parallel case)
    s_num = CondExpLt(den, DEN_THRESHOLD, scalarZero, uTv*vTw - vTv*uTw);
    // Conditional value of s_den (testing parallel case)
    s_den = CondExpLt(den, DEN_THRESHOLD, scalarOne, den);
    // Conditional value of t_num (testing parallel case)
    t_num = CondExpLt(den, DEN_THRESHOLD, vTw, uTu*vTw - uTv*uTw);
    // Conditional value of t_den (testing parallel case)
    t_den = CondExpLt(den, DEN_THRESHOLD, vTv, den);

    // Check the constraint s in [0,1]
    // Conditional value of t_num (testing constraint on s)
    t_num = CondExpLt(s_num, scalarZero, vTw, t_num);
    t_num = CondExpGt(s_num, s_den, vTw + uTv, t_num);

    // Conditional value of t_den (testing constraint on s)
    t_den = CondExpLt(s_num, scalarZero, vTv, t_den);
    t_den = CondExpGt(s_num, s_den, vTv, t_den);

    // Conditional value of s_num (testing constraint on s)
    s_num = CondExpLt(s_num, scalarZero, scalarZero, s_num);
    s_num = CondExpGt(s_num, s_den, s_den, s_num);

    // Check the constraint t in [0,1]
    // Decision tree for s_num
    Scalar s_num_A;
    Scalar s_num_B;
    Scalar s_num_C;
    Scalar s_num_D;
    Scalar s_num_E;
    s_num_A = CondExpGt(-uTw + uTv, uTu, s_den, -uTw + uTv);
    s_num_B = CondExpLt(-uTw + uTv, scalarZero, scalarZero, s_num_A);
    s_num_C = CondExpGt(t_num, t_den, s_num_B, s_num);
    s_num_D = CondExpGt(-uTw, uTu, s_den, -uTw);
    s_num_E = CondExpLt(-uTw, scalarZero, scalarZero, s_num_D);

    s_num = CondExpLt(t_num, scalarZero, s_num_E, s_num_C);

    // Decision tree for s_den
    Scalar s_den_A;
    Scalar s_den_B;
    Scalar s_den_C;
    Scalar s_den_D;
    Scalar s_den_E;
    s_den_A = CondExpGt(-uTw + uTv, uTu, s_den, uTu);
    s_den_B = CondExpLt(-uTw + uTv, scalarZero, s_den, s_den_A);
    s_den_C = CondExpGt(t_num, t_den, s_den_B, s_den);
    s_den_D = CondExpGt(-uTw, uTu, s_den, uTu);
    s_den_E = CondExpLt(-uTw, scalarZero, s_den, s_den_D);

    s_den = CondExpLt(t_num, scalarZero, s_den_E, s_den_C);

    // Constrain t between 0 and 1
    t_num = CondExpLt(t_num, scalarZero, scalarZero, t_num);
    t_num = CondExpGt(t_num, t_den, t_den, t_num);

    // Final computation of s_closest, t_closest
    s_closest = CondExpLt(abs(s_num), DEN_THRESHOLD, scalarZero, s_num/s_den);
    t_closest = CondExpLt(abs(t_num), DEN_THRESHOLD, scalarZero, t_num/t_den);

    Scalar diff_at_closest_X = w0 + s_closest*u0 - t_closest*v0;
    Scalar diff_at_closest_Y = w1 + s_closest*u1 - t_closest*v1;
    Scalar diff_at_closest_Z = w2 + s_closest*u2 - t_closest*v2;
    
    Scalar squaredDistanceResult = diff_at_closest_X*diff_at_closest_X + diff_at_closest_Y*diff_at_closest_Y + diff_at_closest_Z*diff_at_closest_Z;
    return squaredDistanceResult;
    }






int main(void) {
    // setting the Scalar template to represent doubles
    typedef double Scalar;
    // use a special object for source code generation
    typedef CG<Scalar> CGScalar;
    typedef AD<CGScalar> ADScalar;
    typedef ADFun<CGScalar> ADFun;

    /***************************************************************************
     *                               the model
     **************************************************************************/
    // Independent vector x (input)
    CppAD::vector<ADScalar> x(12);
    Independent(x);
    // Dependent vector y (output)
    CppAD::vector<ADScalar> y(1);

    ADScalar a = computeDistanceFromSegments<ADScalar>(x[0],x[1],x[2],x[3],x[4],x[5],x[6],x[7],x[8],x[9],x[10],x[11]);
    y[0] = a;

    ADFun fun(x, y); // the model tape   
    //fun.optimize();


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
    
    // Print the C code to the console
    /*std::cout << "// Generated dist(x) :\n";
    std::cout << code.str();
    std::cout << "// Generated dist'(x) :\n";
    std::cout << code_jac.str();
    */

    /* Compile and test the generated code */
    std::string func_name = "seg_seg_dist_cg";
    std::string lib_name = func_name + "_lib";
    // Initialize library
    ModelCSourceGen<Scalar> cgen(fun, func_name);
    cgen.setCreateForwardZero(true); // generates the function 
    cgen.setCreateJacobian(false); // generates the jacobian

    ModelLibraryCSourceGen<Scalar> libcgen(cgen);
    DynamicModelLibraryProcessor<Scalar> dynamicLibManager(libcgen, lib_name);

    // Compile library
    GccCompiler<Scalar> compiler;
    std::vector<std::string> compile_options = compiler.getCompileFlags();
    compile_options[0] = "-Ofast";
    compiler.setCompileFlags(compile_options);
    dynamicLibManager.createDynamicLibrary(compiler, false);
    std::unique_ptr<DynamicLib<Scalar> > dynamicLib;

    // Load library 
    const auto it = dynamicLibManager.getOptions().find("dlOpenMode");
    if(it == dynamicLibManager.getOptions().end()){
        dynamicLib.reset(new LinuxDynamicLib<Scalar>(dynamicLibManager.getLibraryName() + cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION));
    } else {
        int dlOpenMode = std::stoi(it->second);
        dynamicLib.reset(new LinuxDynamicLib<Scalar>(dynamicLibManager.getLibraryName() + cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION, dlOpenMode));
    }

    std::unique_ptr<GenericModel<Scalar> > genFun_ptr = dynamicLib->model(func_name.c_str());

    
    // Evaluate the function on an example :
    std::vector<Scalar> test_x(12);
    test_x[0] = -1;
    test_x[1] = 2;
    test_x[2] = 0.5;

    test_x[3] = 1;
    test_x[4] = 1.2;
    test_x[5] = 0.7;
    
    test_x[6] = 0.2;
    test_x[7] = -1;
    test_x[8] = 2.5;
    
    test_x[9] = -4;
    test_x[10] = 2.2;
    test_x[11] = 3;

    std::vector<Scalar> test_y(1);

    // Original result
    Scalar original_result = computeDistanceFromPoints<Scalar>(test_x[0],test_x[1],test_x[2],test_x[3],test_x[4],test_x[5],test_x[6],test_x[7],test_x[8],test_x[9],test_x[10],test_x[11]);
    genFun_ptr->ForwardZero(test_x, test_y);

    std::cout << "Original result : " << original_result << std::endl;
    std::cout << "CG result : " << test_y[0] << std::endl;
}