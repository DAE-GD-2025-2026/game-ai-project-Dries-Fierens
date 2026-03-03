#pragma once
#include <vector>

template <typename T>
class MemoryPool {
public:
    MemoryPool(int size, int& count) : m_Pool(size), m_Count(count) {}

    T* allocate() {
        if (m_Count < m_Pool.size()) {
            return &m_Pool[m_Count++];
        }
        return nullptr;
    }

    void reset() {
        m_Count = 0;
    }

    int size() const {
        return m_Pool.size();
    }

    const std::vector<T>& GetPool() const {
        return m_Pool;
    }

	void SetPool(const std::vector<T>& pool) {
		m_Pool = pool;
	}

private:
    std::vector<T> m_Pool;
    int& m_Count;
};
