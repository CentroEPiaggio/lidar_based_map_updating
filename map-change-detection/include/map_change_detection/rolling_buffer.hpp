#pragma once

/*
	This class implements the rolling buffer that stores the 'changed' and 'unchanged' flags.
	A 'changed' flag corresponds to a boolean true, while 'unchanged' corresponds to a boolean false.

	This class uses an array instead of a vector. It's such a big pain, but it allows saving A LOT of memory (more than 100MB in the parking lot environment)
*/

class RollingBuffer
{
	public:
		RollingBuffer();
		RollingBuffer(const int& n);
		~RollingBuffer() { delete flags; }

		inline const int8_t& getCount() const	{	return count;	}

		inline void setSize(const int& n);
		inline const int8_t& getSize() const 	{	return size;	}

		/** Insert elem inside the buffer and update the count */
		void insert(const bool& elem);

	private:
		int8_t size, idx, count;

		bool* flags;
};

RollingBuffer::RollingBuffer() : idx(0), count(0), size(0)
{};

RollingBuffer::RollingBuffer(const int& n) : size(n), idx(0), count(0)
{
	flags = new bool[n];
	for(int i = 0; i < n; i++)
		flags[i] = false;
}

void RollingBuffer::setSize(const int& n)	{
	if(size!=0)
		return;
	size = n;
	flags = new bool[n];
	for(int i = 0; i < size; i++)
		flags[i] = false;
}

void RollingBuffer::insert(const bool& elem) {
	if(elem != flags[idx]) {
		flags[idx] = elem;
		if(elem)
			count++;
		else
			count--;
	}

	idx++;
	if(idx == size)
		idx = 0;
}