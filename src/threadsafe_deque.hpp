#include <deque>
#include <FreeRTOS.h>
#include <semphr.h>

template <typename T>
class ThreadsafeDeque {
private:
    std::deque<T> data;
    SemaphoreHandle_t counter;
    SemaphoreHandle_t mux;

public:
    ThreadsafeDeque(size_t deque_size = 1000) {
        counter = xSemaphoreCreateCounting(deque_size, 0);
        mux = xSemaphoreCreateMutex();
    }
    ~ThreadsafeDeque();

    /** @brief Push item to back of the dequeue.
     * 
     * @param item Item to be inserted.
     */
    void pushFront(T item) {
        xSemaphoreTake(mux, portMAX_DELAY);
        data.push_front(item);
        xSemaphoreGive(mux);
        xSemaphoreGive(counter);
    }

    /** @brief Push item to back of the dequeue.
     * 
     * @param item Item to be inserted.
     */
    void pushBack(T item) {
        xSemaphoreTake(mux, portMAX_DELAY);
        data.push_back(item);
        xSemaphoreGive(mux);
        xSemaphoreGive(counter);
    }

    /** @brief Get item count in deque.*/
    size_t count() {
        return uxSemaphoreGetCount(counter);
    }

    /** @brief Pop item from the front of the dequeu and store it in *item.
     * 
     * @param item Pointer to data structure to which to copy poped data.
     * @param timeout timeout in ms when waiting for new data to appear in queue.
     * @return True if data were obtained.
     * False on timeout. 
     */
    bool popFront(T *item, uint32_t timeout = portMAX_DELAY) {
        if (!xSemaphoreTake(counter, timeout))
            return false;
        xSemaphoreTake(mux, portMAX_DELAY);
        *item = data.front();
        data.pop_front();
        xSemaphoreGive(mux);
        return true;
    }

    /** @brief Pop item from the back of the dequeu and store it in *item.
     * 
     * @param item Pointer to data structure to which to copy poped data.
     * @param timeout timeout in ms when waiting for new data to appear in queue.
     * @return True if data were obtained.
     * False on timeout. 
     */
    bool popBack(T *item, uint32_t timeout = portMAX_DELAY) {
        if (!xSemaphoreTake(counter, timeout))
            return false;
        xSemaphoreTake(mux, portMAX_DELAY);
        *item = data.back();
        data.pop_back();
        xSemaphoreGive(mux);
        return true;
    }

};
