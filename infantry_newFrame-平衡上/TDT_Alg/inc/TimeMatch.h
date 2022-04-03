#ifndef __TIME_MATCH_H
#define __TIME_MATCH_H

#include "string.h"
#include <algorithm>
#include "board.h"
#include "my_math.h"

/**
 * @addtogroup TDT_ALG_TIME_MATCH
 * @{
 */

class TimeSimultaneity
{
private:
    void *data_;     ///<储存数据
    int max_size_;   ///<数据块数量
    int tick_size_;  ///<每个数据块最大大小
    int bit_rate_;   ///<串口传输比特率
    int *rcnt_catch; ///<数组，储存每个数据块上次读取位置信息
    int *wcnt_catch; ///<数组，储存每个数据块上次写入位置信息
    int begin = -1;  ///<当前未删除的第一个数据
    int end = 0;     ///<当前未写入的第一个数据
    float time_diff = 0;
    class node_
    {
    private:
        void *buffer_; ///<数据块开始指针
        int size_;     ///<数据块大小
        int *wcnt;     ///<写入位置信息
        int *rcnt;     ///<读取位置信息

    public:
        /**
             * @brief 向缓存里写入数据
             * @tparam a 用于写入的数据
             * @retval node_ 节点
             */
        template <typename T>
        node_ &operator<<(T a)
        {
            if (*wcnt + sizeof(T) > size_)
            {
                // todo:电控错误处理
                return *this;
            }
            memcpy((char *)buffer_ + *wcnt, &a, sizeof(T));
            *wcnt += sizeof(T);
            return *this;
        }

        /**
             * @brief 从缓存里读取数据
             * @tparam a 需要写入的变量
             * @retval node_ 节点
             */
        template <typename T>
        node_ &operator>>(T &a)
        {
            if (*rcnt + sizeof(T) > size_)
            {
                // todo:电控错误处理
                return *this;
            }
            memcpy(&a, (char *)buffer_ + *rcnt, sizeof(T));
            *rcnt += sizeof(T);
            return *this;
        }

        /**
             * @brief 构造函数，依次传入数据开始位置指针，该数据块大小，上次读取位置信息的指针，上次写入位置信息的指针
             * @param[in] buffer 数据开始位置指针
             * @param[in] size 该数据块大小
             * @param[in] r 上次读取位置信息的指针
             * @param[in] w 上次写入位置信息的指针
             */
        node_(void *buffer, int size, int *r, int *w);

        /**
             * @brief 清空该节点的缓存与读写信息
             */
        void clear();

        /**
             * @brief 返回数据块的起始指针
             */
        char *begin() { return (char *)buffer_; };
    };

public:
    /**
         * @brief 构造函数，依次传入数据块数量，每个数据块最大大小，串口比特率
         * @param[in] max_size 数据块数量
         * @param[in] tick_max_size 每个数据块最大大小
         * @param[in] bit_rate 串口比特率
         */
    TimeSimultaneity(int max_size, int tick_max_size, int bit_rate = 460800);

    /**
         * @brief （at创建的数据并不会被top跳过）返回包含索引对应数据的可用于读写的节点
         * @param[in] index 数据索引值
         * @param[in] time 若为正数则填写当前时间，用其打上时间戳；若为0.0f，则检测是否已打上时间戳，若未打上则自动获取时间打上时间戳；若为-1.0f则强行获取当前时间打上时间戳；其他数值则不打上时间戳
         * @note at创建的数据并不会被top跳过,若未top也不会被pop释放，top若获取到at创建的数据不会清空读写信息
         */
    node_ at(int index, float time = 0.0f);

    /**
         * @brief 读取数据到缓存(会清空之间的缓存读写记录)，电控则同时比对时间
         * @param[in] data 串口收到的数据
         * @param[in] dataSize 数据数组的字节数，对于静态数组可以调用sizeof(数组名)
         */
    void readData(char *data, int dataSize);

    /**
         * @brief 将缓存中的数据写出，视觉会同时写出当前时间
         * @param[in] data 要将数据写入的数组
         * @param[in] dataSize 数组的字节数，对于静态数组可以调用sizeof(数组名)
         * @param[in] clear 是否在写入后清空缓存
         */
    void writeData(char *data, int dataSize, bool clear = 0);

    /**
         * @brief 清空缓存以及读写情况与top,pop信息
         */
    void clearCatch();

    /**
         * @brief 弹出一个当前存在的最早存入的数据
         * @note 若不存在则直接return
         */
    void pop();

    /**
         * @brief 占用一个新的数据块并返回其node节点
         * @param[in] time 若为正数则填写当前时间，用其打上时间戳；若为0.0f，则自动获取时间打上时间戳；若为负数则不打时间戳
         * @param[in] cover 若为true则在无节点可用时pop一个节点
         * @return 返回该数据块node节点,若不存在则返回node_(NULL, 0, NULL, NULL)
         */
    node_ top(float time = 0.0f, bool cover = 1);

    /**
         * @brief 通过节点获取索引
         * @param[in] node 数据块节点
         * @return 该数据块所在的索引
         */
    int getIndexByNode(node_ node);

    /**
         * @brief 获取索引所在数据块的时间戳
         * @param[in] index 索引
         * @return 该索引的数据块的时间戳
         */
    float getTimeByIndex(int index);

    /**
     * @brief 获取执行函数时时间进行时间修正
     * @param[in] vision_time 视觉时间
     * @param[in] sizeofSendStruct 发送结构体长度
     * @param[in] sizeofRecvStruct 接受结构体长度
     * @param[in] recvTime 接收时的时间
     */
    void timeFixed(float vision_time, int sizeofSendStruct, int sizeofRecvStruct, float recvTime = 0.0f);

    /**
     * @brief 获得修正后的视觉时间
     * @param[in] stm32_time 电控时间
     * @return 修正后的视觉时间
     */
    float getFixedTime(float stm32_time);

    /**
     * @brief           析构函数，释放内存
     */
    ~TimeSimultaneity();
};

/** @} */

#endif