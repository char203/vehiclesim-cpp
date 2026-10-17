# Applies the project's warning policy to a single target.
#
# Warnings are attached per target (not via global CMAKE_CXX_FLAGS) so that
# third-party code pulled in with FetchContent is not affected.
function(vehiclesim_set_warnings target)
  if(MSVC)
    set(warnings /W4 /permissive-)
    if(VEHICLESIM_WARNINGS_AS_ERRORS)
      list(APPEND warnings /WX)
    endif()
  else()
    set(warnings
      -Wall
      -Wextra
      -Wpedantic
      -Wshadow
      -Wconversion
      -Wsign-conversion
      -Wold-style-cast
      -Wnon-virtual-dtor
      -Woverloaded-virtual
      -Wdouble-promotion
      -Wformat=2)
    if(VEHICLESIM_WARNINGS_AS_ERRORS)
      list(APPEND warnings -Werror)
    endif()
  endif()
  target_compile_options(${target} PRIVATE ${warnings})
endfunction()

# Optional AddressSanitizer + UndefinedBehaviorSanitizer (GCC/Clang only).
function(vehiclesim_enable_sanitizers target)
  if(VEHICLESIM_ENABLE_SANITIZERS AND NOT MSVC)
    target_compile_options(${target} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(${target} PRIVATE -fsanitize=address,undefined)
  endif()
endfunction()
