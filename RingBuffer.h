#pragma once

#include <Arduino.h>

template <typename T, uint16_t cap>
class RingBuffer
{
public:

	RingBuffer()
		:
		m_head(m_buffer),
		m_tail(m_buffer),
		m_size(0),
		m_capacity(cap)
	{}

	void push(T value)
	{
		*m_head++ = value;

		if (full())
		{
			m_tail++;
		}
		else
		{
			m_size++;
		}

		if (m_head == (m_buffer + m_capacity))
		{
			m_head = m_buffer;
		}
		if (m_tail == (m_buffer + m_capacity))
		{
			m_tail = m_buffer;
		}
	}

	void pop()
	{
		if (m_size > 0)
		{
			m_head--;

			if (m_head < m_tail)
			{
				memmove(m_head, m_tail, (m_size - (m_tail - m_buffer)) * sizeof(T));
			}

			m_size--;

			if (m_head == m_buffer)
			{
				m_head = m_buffer + m_size;
			}
			if (m_tail > m_buffer)
			{
				m_tail--;
			}
		}
	}

	void clear()
	{
		m_head = m_buffer;
		m_tail = m_buffer;
		m_size = 0;

		memset(m_buffer, 0, m_capacity);
	}

	const T* head() const
	{
		if (m_head == m_buffer)
		{
			return m_head + m_capacity - 1;
		}
		else
		{
			return m_head - 1;
		}
	}

	const T* tail() const
	{
		return m_tail;
	}

	T* head()
	{
		if (m_head == m_buffer)
		{
			return m_head + m_capacity - 1;
		}
		else
		{
			return m_head - 1;
		}
	}

	T* tail()
	{
		return m_tail;
	}

	const T* next(const T* ptr) const
	{
		++ptr;

		if (ptr == (m_buffer + m_size))
		{
			ptr = m_buffer;
		}

		return ptr;
	}

	const T* previous(const T* ptr) const
	{
		if (m_size > 0)
		{
			if (ptr == m_buffer)
			{
				ptr = m_buffer + m_size;
			}
			
			--ptr;			
		}

		return ptr;
	}

	bool full() const
	{
		return (m_size == m_capacity);
	}

	bool empty() const
	{
		return (m_size == 0);
	}

	uint16_t capacity() const
	{
		return m_capacity;
	}

	uint16_t size() const
	{
		return m_size;
	}

	const T operator[](uint16_t index) const
	{
		return m_buffer[index];
	}

	T operator[](uint16_t index)
	{
		return m_buffer[index];
	}

private:

	T m_buffer[cap];
	T *m_head;
	T *m_tail;
	uint16_t m_size;
	uint16_t m_capacity;
};