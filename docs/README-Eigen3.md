# Eigen-3.3.9 (vs. AP_HAL / ChibiOS) notes.

## Required application #defines

In order for Eigen3 to compile in the ChibiOS/AP_HAL development ecosystem:

In your app wscript file in the bld.ap_program() section make sure
to add 'ALLOW_DOUBLE_MATH_FUNCTIONS' to the defines=[] line:

```
    defines=['ALLOW_DOUBLE_MATH_FUNCTIONS'],
```

If you see compiler errors complaining abou the number of arguments to
the pow() function this is most likely the cause.


## Host build system changes

I am using Fedora 34 (as of when I am plowing through this material)
and I had to modify my fedora cross compiler system slightly.  It
appears to be a conflict between the compiler and the headers, but
this is deep into the std library so I may not be understanding the
root cause or best worksaround.

  In /usr/arm-none-eabi/include/c++/10.2.0/bits/basic_string.h
  - remove std:: from in front of vsnprint() calls


## Ardupilot (waf) build system changes

In ardupilot/Tools/ardupilotwaf/boards.py (chibios section at least),
comment out:

```
    -Werror=float-equal,
```

Eigen3 uses direct float == comparisons, but done in a careful "legit"
way.  The AP build system is rigged to flag these as compile errors,
so we need to turn that off ... but caution on the remainder of your
code because you have just removed a guard rail.


## Eigen3 code changes.

ChibiOS doens't provide std::realloc() and the build environment is
rigged to catch these and generate a compile error.  We need to work
around this.  AP_HAL does provide a realloc() implementation, but I
chose to go another direction because this was a known working
solution:

Implementation for realloc() without std::realloc() in Eigen Memory.h
This function needs to ge added to Eigen's Memory.h file and all
std::realloc() calls need to be removed.

Reference: http://hdiff.luite.com/cgit/eigen/tree/eigen3/Eigen/src/Core/util/Memory.h?id=6507408b52e39c015ac6943bdaa002ebe31c46ce

You can look at Memory.h in this project for reference.

```
/*****************************************************************************
*** Implementation of generic aligned realloc (when no realloc can be used)***
*****************************************************************************/

void* aligned_malloc(std::size_t size);
void  aligned_free(void *ptr);

/** \internal
  * \brief Reallocates aligned memory.
  * Allows reallocation with aligned ptr types. This implementation will
  * always create a new memory chunk and copy the old data.
  */
inline void* generic_aligned_realloc(void* ptr, size_t size, size_t old_size)
{
  if (ptr==0)
    return aligned_malloc(size);

  if (size==0)
  {
    aligned_free(ptr);
    return 0;
  }

  void* newptr = aligned_malloc(size);
  if (newptr == 0)
  {
    #ifdef EIGEN_HAS_ERRNO
    errno = ENOMEM; // according to the standard
    #endif
    return 0;
  }

  if (ptr != 0)
  {
    std::memcpy(newptr, ptr, (std::min)(size,old_size));
    aligned_free(ptr);
  }

  return newptr;
}
```


## Eigen-3.4 (rc1)

I wasn't able to get this too work ... generated strange error
messages.  So I back tracked to Eigen-3.3.9 which is the most recent
stable release.