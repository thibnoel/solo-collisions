#include <iosfwd>
//#include <pinocchio/codegen/cppadcg.hpp>
//#include <pinocchio/parsers/urdf.hpp>
//#include <pinocchio/algorithm/joint-configuration.hpp>
//#include <pinocchio/algorithm/kinematics.hpp>
//#include <pinocchio/algorithm/frames.hpp>
//#include <iostream>

#include "relative-placement-codegen.hpp"

using namespace pinocchio;
using namespace CppAD;
using namespace CppAD::cg;

// Function to generate
// Wrapper for pinocchio::forwardKinematics for frames f1, f2
template<typename Scalar>
SE3Tpl<Scalar> relativePlacement(ModelTpl<Scalar> model, DataTpl<Scalar> data, Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& config, int frameInd1, int frameInd2)
{
    forwardKinematics(model, data, config);
    updateFramePlacements(model, data);
    SE3Tpl<Scalar> oMf1 = data.oMf[frameInd1];
    SE3Tpl<Scalar> oMf2 = data.oMf[frameInd2];
    return oMf1.inverse() * oMf2;
}
// To generate : 
// We want to get : Mrel = Fij(config) 
// in : x (size nq)
// out : M (size 4x4) 

std::string generateCSourceRelativePlacement(Model model, int frameInd1, int frameInd2)
{
    typedef double Scalar;
    // Code gen. specific types
    typedef CppAD::cg::CG<Scalar> CGScalar;
    typedef CppAD::AD<CGScalar> ADScalar;
    typedef CppAD::ADFun<CGScalar> ADFun;

    // Cast the model to ADScalar type and regenerate the model data
    ModelTpl<ADScalar> cast_rmodel = model.cast<ADScalar>(); 
    DataTpl<ADScalar> cast_rdata(cast_rmodel);  

    //*********Setting up code gen variables and function*********
    // Initnialize AD input and output
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_X;
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_Y;
    ad_X.resize(cast_rmodel.nq);
    ad_Y.resize(16);
    CppAD::Independent(ad_X);
    // Initialize AD function
    ADFun ad_fun;
    
    // Record tape
    SE3Tpl<ADScalar> result = relativePlacement<ADScalar>(cast_rmodel, cast_rdata, ad_X, frameInd1, frameInd2);
    ad_Y = Eigen::Map<const Eigen::Matrix<ADScalar, 16, 1> >(result.toHomogeneousMatrix().data(),
                                                         result.toHomogeneousMatrix().size());
    ad_fun.Dependent(ad_X, ad_Y);

    /***************************************************************************
     *                        Generate the C source code
     **************************************************************************/
    CodeHandler<Scalar> handler;

    CppAD::vector<CGScalar> indVars(cast_rmodel.nq); // size of x (Independent variable)
    handler.makeVariables(indVars);

    CppAD::vector<CGScalar> gen_fun = ad_fun.Forward(0, indVars);

    // ??? Set parameters for code generation?
    LanguageC<Scalar> langC("double");
    LangCDefaultVariableNameGenerator<Scalar> nameGen;

    // Generate function code
    std::ostringstream code;
    handler.generateCode(code, langC, gen_fun, nameGen);

    return code.str();
}

/*
int main(void) {
    typedef double Scalar;
    // Code gen. specific types
    typedef CppAD::cg::CG<Scalar> CGScalar;
    typedef CppAD::AD<CGScalar> ADScalar;
    typedef CppAD::ADFun<CGScalar> ADFun;

    //********* Load SOLO 12 model ********************************
    // Setup URDF path
    const std::string urdf_filename = "/opt/openrobots/share/example-robot-data/robots/solo_description/robots/solo12.urdf";

    // Load and build the URDF model
    ModelTpl<Scalar> rmodel;
    pinocchio::urdf::buildModel(urdf_filename,rmodel);
    // Generate model data
    DataTpl<Scalar> rdata(rmodel); 

    // Cast the model to ADScalar type and regenerate the model data
    ModelTpl<ADScalar> cast_rmodel = rmodel.cast<ADScalar>(); 
    DataTpl<ADScalar> cast_rdata(cast_rmodel);   
    
    // Get frames indices from the model
    int fl_upper_leg = rmodel.getFrameId("FL_UPPER_LEG");
    int hr_lower_leg = rmodel.getFrameId("HR_LOWER_LEG");
    int base_link = rmodel.getFrameId("base_link");

    //*********Setting up code gen variables and functions*********
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_X;
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_Y;
    ad_X.resize(cast_rmodel.nq);
    ad_Y.resize(16);
    ADFun ad_fun;
    CppAD::Independent(ad_X);
    
    SE3Tpl<ADScalar> result = relativePlacement<ADScalar>(cast_rmodel, cast_rdata, ad_X, base_link, hr_lower_leg);
    ad_Y = Eigen::Map<const Eigen::Matrix<ADScalar, 16, 1> >(result.toHomogeneousMatrix().data(),
                                                         result.toHomogeneousMatrix().size());
    ad_fun.Dependent(ad_X, ad_Y);*/

    /***************************************************************************
     *                        Generate the C source code
     **************************************************************************/

    /**
     * start the special steps for source code generation for a Jacobian
     */
    /*CodeHandler<Scalar> handler;

    CppAD::vector<CGScalar> indVars(cast_rmodel.nq); // size of x (Independent variable)
    handler.makeVariables(indVars);

    CppAD::vector<CGScalar> gen_fun = ad_fun.Forward(0, indVars);

    // ??? Set parameters for code generation?
    LanguageC<Scalar> langC("double");
    LangCDefaultVariableNameGenerator<Scalar> nameGen;

    // Generate function code
    std::ostringstream code;
    handler.generateCode(code, langC, gen_fun, nameGen);

    // Print the C code to the console
    std::cout << "// Generated rel placement(q) :\n";
    std::cout << code.str();*/

    /***************************************************************************
     *                          Other method : create, compile
     *                          and evaluate the C code on the flight
     **************************************************************************/

    /* Compile and test the generated code */
    /*std::string func_name = "rel_placement_cg";
    std::string lib_name = func_name + "_lib";
    // Initialize library
    ModelCSourceGen<Scalar> cgen(ad_fun, func_name);
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
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> test_x;
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> test_y_cg;

    test_x.resize(rmodel.nq);
    test_x[7] = 1.5;
    test_x[8] = 0.3;
    test_y_cg.resize(16);

    // Original result
    SE3 test_y_or = relativePlacement(rmodel, rdata, test_x, base_link, hr_lower_leg);
    // Generated code result
    genFun_ptr->ForwardZero(test_x, test_y_cg);

    std::cout << "Original result : " << test_y_or << std::endl;
    std::cout << "CG result : "  << std::endl;
    std::cerr<<Eigen::Map<const Eigen::Matrix<Scalar, 4, 4> >(test_y_cg.data())<<std::endl;
}*/