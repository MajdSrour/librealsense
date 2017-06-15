#include "timestamps.h"
#include "sync.h"
#include <algorithm>
#include <iostream>
#include <unistd.h>
using namespace rsimpl;
using namespace std;


concurrent_queue::concurrent_queue() {
    for (int i =0; i < RS_STREAM_COUNT; i++) {
        latest_timestamps[i] = -1;
    }
}

void concurrent_queue::push_back_data(rs_timestamp_data data)
{
    lock_guard<mutex> lock(mtx);

    data_queue.push_back(data);
}

bool concurrent_queue::pop_front_data()
{
    lock_guard<mutex> lock(mtx);

    if (!data_queue.size())
        return false;

    data_queue.pop_front();

    return true;
}

bool concurrent_queue::erase(rs_timestamp_data data)
{
    lock_guard<mutex> lock(mtx);

    auto it = find_if(data_queue.begin(), data_queue.end(),
                      [&](const rs_timestamp_data& element) {
        return (data.frame_number == element.frame_number);
    });

    if (it != data_queue.end())
    {
        data_queue.erase(it);
        return true;
    }

    return false;
}

size_t concurrent_queue::size()
{
    lock_guard<mutex> lock(mtx);

    return data_queue.size();
}

bool concurrent_queue::correct( frame_interface& frame)
{
    lock_guard<mutex> lock(mtx);

    auto it = find_if(data_queue.begin(), data_queue.end(),
                      [&](const rs_timestamp_data& element) {
        return ((frame.get_frame_number() == element.frame_number));
    });


    if (it != data_queue.end())
    {
        if(latest_timestamps[frame.get_stream_type()] != -1 && it->timestamp < latest_timestamps[frame.get_stream_type()]) {
            std::cout << "dropping in order" << std::endl;
            return false;
        }
        double ts = it->timestamp;
        frame.set_timestamp(ts);
        latest_timestamps[frame.get_stream_type()] = ts;
        return true;
    }

    return false;
}

timestamp_corrector::timestamp_corrector(std::atomic<uint32_t>* queue_size, std::atomic<uint32_t>* timeout)
    :event_queue_size(queue_size), events_timeout(timeout)
{
}

timestamp_corrector::~timestamp_corrector()
{
}

void timestamp_corrector::on_timestamp(rs_timestamp_data data)
{
    lock_guard<mutex> lock(mtx);
    if (data_queue[data.source_id].size() <= event_queue_size->load())
        data_queue[data.source_id].push_back_data(data);
    if (data_queue[data.source_id].size() > event_queue_size->load())
        data_queue[data.source_id].pop_front_data();


}

void timestamp_corrector::update_source_id(rs_event_source& source_id, const rs_stream stream)
{
    switch(stream)
    {
    case RS_STREAM_DEPTH:
    case RS_STREAM_COLOR:
    case RS_STREAM_INFRARED:
    case RS_STREAM_INFRARED2:
        source_id = RS_EVENT_IMU_DEPTH_CAM;
        break;
    case RS_STREAM_FISHEYE:
        source_id = RS_EVENT_IMU_MOTION_CAM;
        break;
    default:
        throw std::runtime_error(to_string() << "Unsupported source stream requested " << rs_stream_to_string(stream));
    }
}

bool timestamp_corrector::correct_timestamp(frame_interface& frame, rs_stream stream)
{


    bool res = false;
    rs_event_source source_id;
    update_source_id(source_id, stream);
    unique_lock<mutex> lock(mtx);

    if (!(res = data_queue[source_id].correct(frame)))
    {
        const auto ready = [&]() { return data_queue[source_id].correct(frame); };
        res = cv.wait_for(lock,std::chrono::milliseconds(*events_timeout),ready);

    }
    if (res) {
        frame.set_timestamp_domain(RS_TIMESTAMP_DOMAIN_MICROCONTROLLER);
    }
    return res;
    
}
