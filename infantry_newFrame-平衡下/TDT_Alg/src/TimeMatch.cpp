#include "TimeMatch.h"

/**
 * @ingroup TDT_ALG
 * @defgroup TDT_ALG_TIME_MATCH 视觉时间同步
 * @brief 使用时间同步算法，将视觉时间与单片机时间同步起来，并将陀螺仪的数据产生数据的时间准确发送给视觉，使视觉更好的拟合陀螺仪数据，来准确计算当前电控应该控制的位置，减少抖动
 */

TimeSimultaneity::node_::node_(void *buffer, int size, int *r, int *w) : buffer_(buffer), size_(size), rcnt(r), wcnt(w){};

void TimeSimultaneity::node_::clear()
{
    if (buffer_ == 0)
        return;
    memset(buffer_, 0, size_);
    *wcnt = 0;
    *rcnt = 0;
}

TimeSimultaneity::TimeSimultaneity(int max_size, int tick_max_size, int bit_rate)
{
    time_diff = 0;
    max_size_ = max_size;
    tick_size_ = tick_max_size;
    bit_rate_ = bit_rate;
    data_ = malloc(max_size * tick_max_size + max_size * sizeof(float));
    while (data_ == NULL)
    {
        // todo:电控错误处理
        data_ = malloc(max_size * tick_max_size + max_size * sizeof(float));
    }
    memset(data_, 0, max_size * tick_max_size + max_size * sizeof(float));
    rcnt_catch = new int[max_size_];
    wcnt_catch = new int[max_size_];
    memset(rcnt_catch, 0, max_size_ * sizeof(int));
    memset(wcnt_catch, 0, max_size_ * sizeof(int));
};

TimeSimultaneity::node_ TimeSimultaneity::at(int index, float time)
{
    if (index >= max_size_)
    {
        return TimeSimultaneity::node_((char *)data_ + (max_size_ - 1) * tick_size_, tick_size_, rcnt_catch + (max_size_ - 1), wcnt_catch + (max_size_ - 1));
    }
    float time_in_cache;
    memcpy(&time_in_cache, (char *)data_ + max_size_ * tick_size_ + index * sizeof(float), sizeof(float));
    if ((time == 0.0f && time_in_cache != 0) || time == -1.0f)
    {
        float time_now;
        time_now = getSysTimeUs() / 1e6f;
        time_now = getFixedTime(time_now);
        memcpy((char *)data_ + max_size_ * tick_size_ + index * sizeof(float), &time_now, sizeof(float));
    };
    if (time > 0)
    {
        memcpy((char *)data_ + max_size_ * tick_size_ + index * sizeof(float), &time, sizeof(float));
    }
    return TimeSimultaneity::node_((char *)data_ + index * tick_size_, tick_size_, rcnt_catch + index, wcnt_catch + index);
}

void TimeSimultaneity::readData(char *data, int dataSize)
{
    if (dataSize > max_size_ * tick_size_ + max_size_ * sizeof(float) + 1)
        dataSize = max_size_ * tick_size_ + max_size_ * sizeof(float);
    else
        dataSize--;
    memcpy(data_, data, dataSize);
    memset(rcnt_catch, 0, max_size_ * sizeof(int));
    memset(wcnt_catch, 0, max_size_ * sizeof(int));
    float vision_time;
    memcpy(&vision_time, (char *)data + dataSize, sizeof(float));
    timeFixed(vision_time, 0, 0);
}

void TimeSimultaneity::writeData(char *data, int dataSize, bool clear)
{
    if (dataSize > max_size_ * tick_size_ + max_size_ * sizeof(float))
        dataSize = max_size_ * tick_size_ + max_size_ * sizeof(float);
    memcpy(data, data_, dataSize);
    if (clear)
        clearCatch();
}

void TimeSimultaneity::clearCatch()
{
    memset(data_, 0, max_size_ * tick_size_ + max_size_ * sizeof(float));
    memset(rcnt_catch, 0, max_size_ * sizeof(int));
    memset(wcnt_catch, 0, max_size_ * sizeof(int));
    begin = -1; //当前未删除的第一个数据
    end = 0;    //当前未写入的第一个数据
}

void TimeSimultaneity::pop()
{
    if (begin == -1)
        return;
    memset((char *)data_ + begin * tick_size_, 0, tick_size_);
    memset((char *)data_ + max_size_ * tick_size_ + begin * sizeof(float), 0, tick_size_);
    wcnt_catch[begin] = 0;
    rcnt_catch[begin] = 0;
    if (end == -1)
        end = begin;
    begin++;
    if (begin == max_size_)
        begin = 0;
    if (begin == end)
        begin = -1;
}

TimeSimultaneity::node_ TimeSimultaneity::top(float time, bool cover)
{
    if (end == -1)
    {
        if (cover)
            pop();
        else
            return node_(NULL, 0, NULL, NULL);
    }
    if (time == 0.0f)
    {
        float time_now;
        time_now = getSysTimeUs() / 1e6f;
        time_now = getFixedTime(time_now);
        memcpy((char *)data_ + max_size_ * tick_size_ + end * sizeof(float), &time_now, sizeof(float));
    }
    if (time > 0)
    {
        time = getFixedTime(time);
        memcpy((char *)data_ + max_size_ * tick_size_ + end * sizeof(float), &time, sizeof(float));
    }
    if (begin == -1)
        begin = end;
    int t = end;
    end++;
    if (end == max_size_)
        end = 0;
    if (end == begin)
        end = -1;
    return node_((char *)data_ + t * tick_size_, tick_size_, rcnt_catch + t, wcnt_catch + t);
}

int TimeSimultaneity::getIndexByNode(TimeSimultaneity::node_ node)
{
    int ptr_dis = node.begin() - (char *)data_;
    return ptr_dis / tick_size_;
}

float TimeSimultaneity::getTimeByIndex(int index)
{
    if (index >= max_size_)
    {
        index = max_size_ - 1;
    }
    float time_in_cache;
    memcpy(&time_in_cache, (char *)data_ + max_size_ * tick_size_ + index * sizeof(float), sizeof(float));
    return time_in_cache;
}

void TimeSimultaneity::timeFixed(float vision_time, int sizeofSendStruct, int sizeofRecvStruct, float recvTime)
{
    if (recvTime == 0.0f)
        recvTime = getSysTimeUs();
    recvTime /= 1e6f;
    time_diff = recvTime - vision_time - (float)10.0 * (sizeofSendStruct + sizeofRecvStruct) / bit_rate_;
}

float TimeSimultaneity::getFixedTime(float stm32_time) { return stm32_time - time_diff; }

TimeSimultaneity::~TimeSimultaneity()
{
    free(data_);
    free(rcnt_catch);
    free(wcnt_catch);
}
