set(CMAKE_CXX_COMPILER "/usr/bin/g++-7")
set(CMAKE_CXX_COMPILER_ARG1 "")
set(CMAKE_CXX_COMPILER_ID "GNU")
set(CMAKE_CXX_COMPILER_VERSION "7.5.0")
set(CMAKE_CXX_COMPILER_VERSION_INTERNAL "")
set(CMAKE_CXX_COMPILER_WRAPPER "")
set(CMAKE_CXX_STANDARD_COMPUTED_DEFAULT "14")
set(CMAKE_CXX_EXTENSIONS_COMPUTED_DEFAULT "ON")
set(CMAKE_CXX_COMPILE_FEATURES "cxx_std_98;cxx_template_template_parameters;cxx_std_11;cxx_alias_templates;cxx_alignas;cxx_alignof;cxx_attributes;cxx_auto_type;cxx_constexpr;cxx_decltype;cxx_decltype_incomplete_return_types;cxx_default_function_template_args;cxx_defaulted_functions;cxx_defaulted_move_initializers;cxx_delegating_constructors;cxx_deleted_functions;cxx_enum_forward_declarations;cxx_explicit_conversions;cxx_extended_friend_declarations;cxx_extern_templates;cxx_final;cxx_func_identifier;cxx_generalized_initializers;cxx_inheriting_constructors;cxx_inline_namespaces;cxx_lambdas;cxx_local_type_template_args;cxx_long_long_type;cxx_noexcept;cxx_nonstatic_member_init;cxx_nullptr;cxx_override;cxx_range_for;cxx_raw_string_literals;cxx_reference_qualified_functions;cxx_right_angle_brackets;cxx_rvalue_references;cxx_sizeof_member;cxx_static_assert;cxx_strong_enums;cxx_thread_local;cxx_trailing_return_types;cxx_unicode_literals;cxx_uniform_initialization;cxx_unrestricted_unions;cxx_user_literals;cxx_variadic_macros;cxx_variadic_templates;cxx_std_14;cxx_aggregate_default_initializers;cxx_attribute_deprecated;cxx_binary_literals;cxx_contextual_conversions;cxx_decltype_auto;cxx_digit_separators;cxx_generic_lambdas;cxx_lambda_init_captures;cxx_relaxed_constexpr;cxx_return_type_deduction;cxx_variable_templates;cxx_std_17")
set(CMAKE_CXX98_COMPILE_FEATURES "cxx_std_98;cxx_template_template_parameters")
set(CMAKE_CXX11_COMPILE_FEATURES "cxx_std_11;cxx_alias_templates;cxx_alignas;cxx_alignof;cxx_attributes;cxx_auto_type;cxx_constexpr;cxx_decltype;cxx_decltype_incomplete_return_types;cxx_default_function_template_args;cxx_defaulted_functions;cxx_defaulted_move_initializers;cxx_delegating_constructors;cxx_deleted_functions;cxx_enum_forward_declarations;cxx_explicit_conversions;cxx_extended_friend_declarations;cxx_extern_templates;cxx_final;cxx_func_identifier;cxx_generalized_initializers;cxx_inheriting_constructors;cxx_inline_namespaces;cxx_lambdas;cxx_local_type_template_args;cxx_long_long_type;cxx_noexcept;cxx_nonstatic_member_init;cxx_nullptr;cxx_override;cxx_range_for;cxx_raw_string_literals;cxx_reference_qualified_functions;cxx_right_angle_brackets;cxx_rvalue_references;cxx_sizeof_member;cxx_static_assert;cxx_strong_enums;cxx_thread_local;cxx_trailing_return_types;cxx_unicode_literals;cxx_uniform_initialization;cxx_unrestricted_unions;cxx_user_literals;cxx_variadic_macros;cxx_variadic_templates")
set(CMAKE_CXX14_COMPILE_FEATURES "cxx_std_14;cxx_aggregate_default_initializers;cxx_attribute_deprecated;cxx_binary_literals;cxx_contextual_conversions;cxx_decltype_auto;cxx_digit_separators;cxx_generic_lambdas;cxx_lambda_init_captures;cxx_relaxed_constexpr;cxx_return_type_deduction;cxx_variable_templates")
set(CMAKE_CXX17_COMPILE_FEATURES "cxx_std_17")
set(CMAKE_CXX20_COMPILE_FEATURES "")
set(CMAKE_CXX23_COMPILE_FEATURES "")

set(CMAKE_CXX_PLATFORM_ID "Linux")
set(CMAKE_CXX_SIMULATE_ID "")
set(CMAKE_CXX_COMPILER_FRONTEND_VARIANT "GNU")
set(CMAKE_CXX_SIMULATE_VERSION "")




set(CMAKE_AR "/usr/bin/ar")
set(CMAKE_CXX_COMPILER_AR "/usr/bin/gcc-ar-7")
set(CMAKE_RANLIB "/usr/bin/ranlib")
set(CMAKE_CXX_COMPILER_RANLIB "/usr/bin/gcc-ranlib-7")
set(CMAKE_LINKER "/usr/bin/ld")
set(CMAKE_MT "")
set(CMAKE_TAPI "CMAKE_TAPI-NOTFOUND")
set(CMAKE_COMPILER_IS_GNUCXX 1)
set(CMAKE_CXX_COMPILER_LOADED 1)
set(CMAKE_CXX_COMPILER_WORKS TRUE)
set(CMAKE_CXX_ABI_COMPILED TRUE)

set(CMAKE_CXX_COMPILER_ENV_VAR "CXX")

set(CMAKE_CXX_COMPILER_ID_RUN 1)
set(CMAKE_CXX_SOURCE_FILE_EXTENSIONS C;M;c++;cc;cpp;cxx;m;mm;mpp;CPP;ixx;cppm;ccm;cxxm;c++m)
set(CMAKE_CXX_IGNORE_EXTENSIONS inl;h;hpp;HPP;H;o;O;obj;OBJ;def;DEF;rc;RC)

foreach (lang C OBJC OBJCXX)
  if (CMAKE_${lang}_COMPILER_ID_RUN)
    foreach(extension IN LISTS CMAKE_${lang}_SOURCE_FILE_EXTENSIONS)
      list(REMOVE_ITEM CMAKE_CXX_SOURCE_FILE_EXTENSIONS ${extension})
    endforeach()
  endif()
endforeach()

set(CMAKE_CXX_LINKER_PREFERENCE 30)
set(CMAKE_CXX_LINKER_PREFERENCE_PROPAGATES 1)
set(CMAKE_CXX_LINKER_DEPFILE_SUPPORTED TRUE)

# Save compiler ABI information.
set(CMAKE_CXX_SIZEOF_DATA_PTR "8")
set(CMAKE_CXX_COMPILER_ABI "ELF")
set(CMAKE_CXX_BYTE_ORDER "LITTLE_ENDIAN")
set(CMAKE_CXX_LIBRARY_ARCHITECTURE "")

if(CMAKE_CXX_SIZEOF_DATA_PTR)
  set(CMAKE_SIZEOF_VOID_P "${CMAKE_CXX_SIZEOF_DATA_PTR}")
endif()

if(CMAKE_CXX_COMPILER_ABI)
  set(CMAKE_INTERNAL_PLATFORM_ABI "${CMAKE_CXX_COMPILER_ABI}")
endif()

if(CMAKE_CXX_LIBRARY_ARCHITECTURE)
  set(CMAKE_LIBRARY_ARCHITECTURE "")
endif()

set(CMAKE_CXX_CL_SHOWINCLUDES_PREFIX "")
if(CMAKE_CXX_CL_SHOWINCLUDES_PREFIX)
  set(CMAKE_CL_SHOWINCLUDES_PREFIX "${CMAKE_CXX_CL_SHOWINCLUDES_PREFIX}")
endif()





set(CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES "/usr/include/c++/7;/usr/include/c++/7/x86_64-suse-linux;/usr/include/c++/7/backward;/usr/lib64/gcc/x86_64-suse-linux/7/include;/usr/local/include;/usr/lib64/gcc/x86_64-suse-linux/7/include-fixed;/usr/x86_64-suse-linux/include;/usr/include")
set(CMAKE_CXX_IMPLICIT_LINK_LIBRARIES "stdc++;m;gcc_s;gcc;pthread;c;gcc_s;gcc")
set(CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES "/volume/conan_cache/giub_ri/.conan/data/pangolin/0.9.1/3rdparty/stable/package/6c8cbd78feb3c7902eee79aede57a597ead6e2b1/lib;/volume/conan_cache/giub_ri/.conan/data/ceres-solver/2.1.0/3rdparty/stable/package/f1b6a847b0b5e17b6ad36d16106b2a67705d012c/lib;/volume/conan_cache/giub_ri/.conan/data/visual-lcd/1.5.0/moro/snapshot/package/c51c4ed812e1492d51cff3eb5e95ebc37d518861/lib;/volume/conan_cache/giub_ri/.conan/data/ros_core/1.4.1/3rdparty/stable/package/dfc56f6663a878aad7ca79ac3af67159360b6d1b/lib;/volume/conan_cache/giub_ri/.conan/data/opencv/4.9.0/3rdparty/unstable/package/ca04e94a2870386cd14b7f4f0cb10613e95a79c6/lib;/volume/conan_cache/giub_ri/.conan/data/boost/1.66.0/3rdparty/stable/package/0951097891faa9d39de5771af51dcfcb1d49dfdc/lib;/volume/conan_cache/giub_ri/.conan/data/console_bridge/0.4.4/3rdparty/stable/package/a059740f98a1247ec031a04c0fc4bf6c5a67a0fa/lib;/volume/conan_cache/giub_ri/.conan/data/tinyxml/2.6.2/3rdparty/stable/package/1c6d59c8485ce7ba010ce9f8f3b718e1743ee65e/lib;/volume/conan_cache/giub_ri/.conan/data/lz4/1.9.2/3rdparty/stable/package/7566a2e36235aaa167cd309b32e63e74379fa88a/lib;/volume/conan_cache/giub_ri/.conan/data/log4cxx/0.10.0/3rdparty/stable/package/7566a2e36235aaa167cd309b32e63e74379fa88a/lib;/volume/conan_cache/giub_ri/.conan/data/rospkg/1.4.0/pypi/stable/package/9617b349fafb3f9addb24517a8678125c3405ab1/lib;/volume/conan_cache/giub_ri/.conan/data/poco/1.9.4/3rdparty/stable/package/cadd63bbdaec45074f45011064cede7725fb0d8e/lib;/volume/conan_cache/giub_ri/.conan/data/defusedxml/0.6.0/pypi/stable/package/5eda30965d7a60fa3a1033f7940c94dc73878702/lib;/volume/conan_cache/giub_ri/.conan/data/pycryptodomex/3.9.8/pypi/stable/package/5eda30965d7a60fa3a1033f7940c94dc73878702/lib;/volume/conan_cache/giub_ri/.conan/data/gnupg/2.3.1/pypi/stable/package/5eda30965d7a60fa3a1033f7940c94dc73878702/lib;/volume/conan_cache/giub_ri/.conan/data/catkin/0.8.1/3rdparty/stable/package/f3ba25740b99839924b50cc790bd62e6ef5124bf/lib;/volume/conan_cache/giub_ri/.conan/data/eigen/3.3.7/3rdparty/stable/package/66732ee6185ec10ba26fe5f063a64aba30f472ea/lib;/volume/conan_cache/giub_ri/.conan/data/glog/0.4.0/3rdparty/stable/package/cadd63bbdaec45074f45011064cede7725fb0d8e/lib;/volume/conan_cache/giub_ri/.conan/data/gflags/2.2.2/3rdparty/stable/package/e096be2ccc3d13b81e10eae858042b5e6734f08c/lib;/volume/conan_cache/giub_ri/.conan/data/openexr/2.5.3/3rdparty/stable/package/72efab4638fb46dbb3bb6c6e83c29693282f1981/lib;/volume/conan_cache/giub_ri/.conan/data/tbb/2019.8.0/3rdparty/stable/package/3b9e4c5f9dd918c27eedc8a5e14627e1abc0cd76/lib;/volume/conan_cache/giub_ri/.conan/data/gtest/1.10.0/3rdparty/stable/package/36f4faa8f490b828f0bb0af88ed9ae0a6a12248f/lib;/volume/conan_cache/giub_ri/.conan/data/catkin-tools/0.9.0/pypi/stable/package/e19ebe9a9867c1c00eb19479adaba9d411102a8c/lib;/volume/conan_cache/giub_ri/.conan/data/catkin-pkg/1.0.0/pypi/stable/package/5eda30965d7a60fa3a1033f7940c94dc73878702/lib;/volume/conan_cache/giub_ri/.conan/data/osrf-pycommon/2.0.2/pypi/stable/package/5eda30965d7a60fa3a1033f7940c94dc73878702/lib;/opt/python/osl155-x86_64/python3/stable/2.2.1/lib;/usr/lib64/gcc/x86_64-suse-linux/7;/usr/lib64;/lib64;/usr/x86_64-suse-linux/lib")
set(CMAKE_CXX_IMPLICIT_LINK_FRAMEWORK_DIRECTORIES "")
