#ifndef _MALLOC_ALLOCATOR_H
#define _MALLOC_ALLOCATOR_H

#include <limits.h>
#include <stddef.h>
#ifdef STM32
#include "malloc.h"
#endif

template <class T> 
class malloc_allocator
{
public:
	typedef T                 value_type;
	typedef value_type*       pointer;
	typedef const value_type* const_pointer;
	typedef value_type&       reference;
	typedef const value_type& const_reference;
	typedef size_t       size_type;
	typedef ptrdiff_t    difference_type;

	template <class U>
	struct rebind { typedef malloc_allocator<U> other; };

	malloc_allocator() {}
	malloc_allocator(const malloc_allocator&) {}

	template <class U>
	malloc_allocator(const malloc_allocator<U>&) {}
	~malloc_allocator() {}

	pointer address(reference x) const { return &x; }
	const_pointer address(const_reference x) const {
		return x;
	}

	pointer allocate(size_type n, const_pointer = 0) {
#ifdef STM32
		void* p = malloc(n * sizeof(T));
#else
		void* p = std::malloc(n * sizeof(T));
#endif
		if (!p)
			throw std::bad_alloc();
		return static_cast<pointer>(p);
	}

#ifdef STM32
	void deallocate(pointer p, size_type) { free(p); }
#else
	void deallocate(pointer p, size_type) { std::free(p); }
#endif

	size_type max_size() const {
		return static_cast<size_type>(-1) / sizeof(T);
	}

	void construct(pointer p, const value_type& x) {
		new(p) value_type(x);
	}

	void destroy(pointer p) { p->~value_type(); }

private:

	void operator=(const malloc_allocator&);
};

template<> 
class malloc_allocator<void>
{
	typedef void        value_type;
	typedef void*       pointer;
	typedef const void* const_pointer;

	template <class U>
	struct rebind { typedef malloc_allocator<U> other; };
};

template <class T>
inline bool operator==(const malloc_allocator<T>&,
	const malloc_allocator<T>&) {
	return true;
}

template <class T>
inline bool operator!=(const malloc_allocator<T>&,
	const malloc_allocator<T>&) {
	return false;
}
#endif
