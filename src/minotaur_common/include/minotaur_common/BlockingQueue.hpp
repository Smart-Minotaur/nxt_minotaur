#ifndef MINOTAUR_BLOCKING_QUEUE_HPP
#define MINOTAUR_BLOCKING_QUEUE_HPP

#include <deque>
#include <pthread.h>

namespace minotaur
{
    /**
     * \brief The BlockingQueue class implements a blocking queue with
     *        dynamic size using the pthread API.
     */
    template <typename T> class BlockingQueue
    {
    private:
        pthread_mutex_t mutex;
        pthread_cond_t popCondition;
        std::deque<T> queue;
    public:
        BlockingQueue();
        virtual ~BlockingQueue();
        
        void enqueue(const T &p_obj);
        T dequeue();
    };
    
    template <typename T>
    BlockingQueue<T>::BlockingQueue()
    {
        pthread_cond_init(&popCondition, NULL);
        pthread_mutex_init(&mutex, NULL);
    }
    
    template <typename T>
    BlockingQueue<T>::~BlockingQueue()
    {
        pthread_mutex_destroy(&mutex);
        pthread_cond_destroy(&popCondition);
    }
    
    template <typename T>
    void BlockingQueue<T>::enqueue(const T &p_obj)
    {
        pthread_mutex_lock(&mutex);
	
        queue.push_back(p_obj);
	
        pthread_mutex_unlock(&mutex);
        pthread_cond_signal(&popCondition);
    }
    
    template <typename T>
    T BlockingQueue<T>::dequeue()
    {
        pthread_mutex_lock(&mutex);
        while(queue.empty())
            pthread_cond_wait(&popCondition, &mutex);
        
        T result = queue.front();
        queue.pop_front();
	
        pthread_mutex_unlock(&mutex);
	
	return result;
    }
}

#endif
