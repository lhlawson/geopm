/*
 * Copyright (c) 2015, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY LOG OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <float.h>
#include <hwloc.h>

#include "geopm.h"
#include "geopm_sched.h"
#include "geopm_message.h"
#include "geopm_time.h"
#include "Profile.hpp"
#include "Exception.hpp"
#include "LockingHashTable.hpp"

extern "C"
{
    int geopm_prof_create(const char *name, size_t table_size, const char *shm_key, MPI_Comm comm, struct geopm_prof_c **prof)
    {
        int err = 0;
        try {
            *prof = (struct geopm_prof_c *)(new geopm::Profile(std::string(name), table_size, std::string(shm_key), comm));
        }
        catch (...) {
            err = geopm::exception_handler(std::current_exception());
        }
        return err;
    }

    int geopm_prof_destroy(struct geopm_prof_c *prof)
    {
        int err = 0;
        try {
            geopm::Profile *prof_obj = (geopm::Profile *)prof;
            if (prof_obj == NULL) {
                throw geopm::Exception(GEOPM_ERROR_PROF_NULL, __FILE__, __LINE__);
            }
            delete prof_obj;
        }
        catch (...) {
            err = geopm::exception_handler(std::current_exception());
        }
        return err;
    }

    int geopm_prof_region(struct geopm_prof_c *prof, const char *region_name, long policy_hint, uint64_t *region_id)
    {
        int err = 0;
        try {
            geopm::Profile *prof_obj = (geopm::Profile *)prof;
            if (prof_obj == NULL) {
                throw geopm::Exception(GEOPM_ERROR_PROF_NULL, __FILE__, __LINE__);
            }
            *region_id = prof_obj->region(std::string(region_name), policy_hint);
        }
        catch (...) {
            err = geopm::exception_handler(std::current_exception());
        }
        return err;
    }

    int geopm_prof_enter(struct geopm_prof_c *prof, uint64_t region_id)
    {
        int err = 0;
        try {
            geopm::Profile *prof_obj = (geopm::Profile *)prof;
            if (prof_obj == NULL) {
                throw geopm::Exception(GEOPM_ERROR_PROF_NULL, __FILE__, __LINE__);
            }
            prof_obj->enter(region_id);
        }
        catch (...) {
            err = geopm::exception_handler(std::current_exception());
        }
        return err;

    }

    int geopm_prof_exit(struct geopm_prof_c *prof, uint64_t region_id)
    {
        int err = 0;
        try {
            geopm::Profile *prof_obj = (geopm::Profile *)prof;
            if (prof_obj == NULL) {
                throw geopm::Exception(GEOPM_ERROR_PROF_NULL, __FILE__, __LINE__);
            }
            prof_obj->exit(region_id);
        }
        catch (...) {
            err = geopm::exception_handler(std::current_exception());
        }
        return err;

    }

    int geopm_prof_progress(struct geopm_prof_c *prof, uint64_t region_id, double fraction)
    {
        int err = 0;
        try {
            geopm::Profile *prof_obj = (geopm::Profile *)prof;
            if (prof_obj == NULL) {
                throw geopm::Exception(GEOPM_ERROR_PROF_NULL, __FILE__, __LINE__);
            }
            prof_obj->progress(region_id, fraction);
        }
        catch (...) {
            err = geopm::exception_handler(std::current_exception());
        }
        return err;

    }

    int geopm_prof_outer_sync(struct geopm_prof_c *prof)
    {
        int err = 0;
        try {
            geopm::Profile *prof_obj = (geopm::Profile *)prof;
            if (prof_obj == NULL) {
                throw geopm::Exception(GEOPM_ERROR_PROF_NULL, __FILE__, __LINE__);
            }
            prof_obj->outer_sync();
        }
        catch (...) {
            err = geopm::exception_handler(std::current_exception());
        }
        return err;

    }

    int geopm_prof_sample(struct geopm_prof_c *prof, uint64_t region_id)
    {
        int err = 0;
        try {
            geopm::Profile *prof_obj = (geopm::Profile *)prof;
            if (prof_obj == NULL) {
                throw geopm::Exception(GEOPM_ERROR_PROF_NULL, __FILE__, __LINE__);
            }
            prof_obj->sample(region_id);
        }
        catch (...) {
            err = geopm::exception_handler(std::current_exception());
        }
        return err;

    }

    int geopm_prof_enable(struct geopm_prof_c *prof, const char *feature_name)
    {
        int err = 0;
        try {
            geopm::Profile *prof_obj = (geopm::Profile *)prof;
            if (prof_obj == NULL) {
                throw geopm::Exception(GEOPM_ERROR_PROF_NULL, __FILE__, __LINE__);
            }
            prof_obj->enable(std::string(feature_name));
        }
        catch (...) {
            err = geopm::exception_handler(std::current_exception());
        }
        return err;

    }

    int geopm_prof_disable(struct geopm_prof_c *prof, const char *feature_name)
    {
        int err = 0;
        try {
            geopm::Profile *prof_obj = (geopm::Profile *)prof;
            if (prof_obj == NULL) {
                throw geopm::Exception(GEOPM_ERROR_PROF_NULL, __FILE__, __LINE__);
            }
            prof_obj->disable(std::string(feature_name));
        }
        catch (...) {
            err = geopm::exception_handler(std::current_exception());
        }
        return err;

    }

    int geopm_prof_print(struct geopm_prof_c *prof, const char *file_name, int depth)
    {
        int err = 0;
        try {
            geopm::Profile *prof_obj = (geopm::Profile *)prof;
            if (prof_obj == NULL) {
                throw geopm::Exception(GEOPM_ERROR_PROF_NULL, __FILE__, __LINE__);
            }
            prof_obj->print(std::string(file_name), depth);
        }
        catch (...) {
            err = geopm::exception_handler(std::current_exception());
        }
        return err;

    }

    double geopm_progress_threaded_min(int num_thread,
                                       size_t stride,
                                       const uint32_t *progress,
                                       const double *norm)
    {
        double progress_min = DBL_MAX;
        double progress_tmp;
        int j;

        for (j = 0; j < num_thread; ++j) {
            progress_tmp = progress[j * stride] * norm[j];
            progress_min =  progress_tmp < progress_min ?
                            progress_tmp : progress_min;
        }
        return progress_min;
    }

    int geopm_omp_sched_static_norm(int num_iter, int chunk_size, int num_thread, double *norm)
    {
        int remain = num_iter;
        int i = 0;

        /* inefficient but robust way of calculating the norm based on
           OpenMP documentation. */
        memset(norm, 0, sizeof(double) * num_thread);
        while (remain) {
            if (remain > chunk_size) {
                norm[i] += chunk_size;
                remain -= chunk_size;
            }
            else {
                norm[i] += remain;
                remain = 0;
            }
            i++;
            if (i == num_thread) {
                i = 0;
            }
        }
        for (i = 0; i < num_thread; ++i) {
            if (norm[i] != 0.0) {
                norm[i] = 1.0 / norm[i];
            }
        }
        return 0;
    }
}

namespace geopm
{
    Profile::Profile(const std::string prof_name, size_t table_size, const std::string shm_key, MPI_Comm comm)
        : m_prof_name(prof_name)
        , m_curr_region_id(0)
        , m_enter_time({{0, 0}})
        , m_num_enter(0)
        , m_num_progress(0)
        , m_progress(0.0)
    {
        int rank;

        MPI_Comm_rank(comm, &rank);
        MPI_Comm_split_type(comm, MPI_COMM_TYPE_SHARED, rank, MPI_INFO_NULL, &m_shm_comm);
        MPI_Comm_rank(m_shm_comm, &m_shm_rank);
        m_table_shmem = new SharedMemoryUser(shm_key, table_size);
        m_table_buffer = m_table_shmem->pointer();
        m_table = new LockingHashTable<struct geopm_sample_message_s>(table_size, m_table_buffer);
        init_cpu_list();
        for (auto it = m_cpu_list.begin(); it != m_cpu_list.end(); ++it) {
            m_ctl_msg->cpu_rank[*it] = rank;
        }
        MPI_Barrier(m_shm_comm);
        if (!m_shm_rank) {
            m_ctl_msg->app_status = GEOPM_STATUS_INITIALIZED;
        }
    }

    Profile::~Profile()
    {
        delete m_table;
        delete m_table_shmem;
    }

    uint64_t Profile::region(const std::string region_name, long policy_hint)
    {
        return m_table->key(region_name);
        // FIXME: not using policy_hint
    }

    void Profile::enter(uint64_t region_id)
    {
        if (!m_curr_region_id) {
            m_curr_region_id = region_id;
            m_num_enter = 0;
            (void) geopm_time(&m_enter_time);
        }
        if (m_curr_region_id == region_id) {
            ++m_num_enter;
        }
    }

    void Profile::exit(uint64_t region_id)
    {
        if (m_curr_region_id == region_id) {
            --m_num_enter;
        }
        if (!m_num_enter) {
            struct geopm_sample_message_s sample = m_table->find(region_id);
            sample.region_id = region_id;
            sample.progress = 1.0;
            struct geopm_time_s exit_time;
            (void) geopm_time(&exit_time);
            sample.runtime = geopm_time_diff(&m_enter_time, &exit_time);
            m_table->insert(region_id, sample);
            m_curr_region_id = 0;
        }
    }

    void Profile::progress(uint64_t region_id, double fraction)
    {
        if (m_num_enter == 1 && m_curr_region_id == region_id) {
            m_progress = fraction;
            ++m_num_progress;
            if (m_num_progress == GEOPM_CONST_PROF_SAMPLE_PERIOD) {
                sample(region_id);
                m_num_progress = 0;
            }
        }
    }

    void Profile::outer_sync(void)
    {
        throw geopm::Exception("Profile::outer_sync()", GEOPM_ERROR_NOT_IMPLEMENTED, __FILE__, __LINE__);
    }

    void Profile::sample(uint64_t region_id)
    {
        if (region_id == m_curr_region_id) {
            struct geopm_sample_message_s sample;
            struct geopm_time_s curr_time;
            sample.region_id = region_id;
            (void) geopm_time(&curr_time);
            sample.runtime = geopm_time_diff(&m_enter_time, &curr_time);
            sample.progress = m_progress;
            sample.energy = 0.0;     // FIXME: need to add a platform to Profile and
            sample.frequency = 0.0;  // update energy and frequency here.
            m_table->insert(region_id, sample);
        }
    }

    void Profile::enable(const std::string feature_name)
    {
        throw geopm::Exception("Profile::enable()", GEOPM_ERROR_NOT_IMPLEMENTED, __FILE__, __LINE__);
    }

    void Profile::disable(const std::string feature_name)
    {
        throw geopm::Exception("Profile::disable()", GEOPM_ERROR_NOT_IMPLEMENTED, __FILE__, __LINE__);
    }

    void Profile::print(const std::string file_name, int depth)
    {
        int is_done = 0;
        int all_done = 0;
        MPI_Barrier(m_shm_comm);
        if (!m_shm_rank) {
            m_ctl_msg->app_status = GEOPM_STATUS_REPORT;
        }
        while (m_ctl_msg->ctl_status != GEOPM_STATUS_REPORT) {}
        size_t buffer_offset = 0;
        char *buffer_ptr = (char *)(m_table_shmem->pointer());

        if (GEOPM_CONST_SHMEM_REGION_SIZE < file_name.length() + 1 + m_prof_name.length() + 1) {
            throw Exception("Profile:print() profile file name and profile name are too long to fit in a table buffer", GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
        strcpy(buffer_ptr, file_name.c_str());
        buffer_ptr += file_name.length() + 1;
        buffer_offset += file_name.length() + 1;
        strcpy(buffer_ptr, m_prof_name.c_str());
        buffer_offset += m_prof_name.length() + 1;
        while (!all_done) {
            is_done = m_table->name_fill(buffer_offset);
            MPI_Reduce(&is_done, &all_done, 1, MPI_INT, MPI_LAND, 0, m_shm_comm);
            if (!m_shm_rank) {
                m_ctl_msg->app_status = GEOPM_STATUS_READY;
            }
            while (m_ctl_msg->ctl_status != GEOPM_STATUS_READY) {}
            MPI_Barrier(m_shm_comm);
            if (!m_shm_rank && !all_done) {
                m_ctl_msg->app_status = GEOPM_STATUS_REPORT;
            }
            buffer_offset = 0;
        }
        if (!m_shm_rank) {
            m_ctl_msg->app_status = GEOPM_STATUS_SHUTDOWN;
        }
    }

    void Profile::init_cpu_list(void)
    {
        int err = 0;
        unsigned int i = 0;

        hwloc_topology_t topology;
        hwloc_cpuset_t set;

        err = hwloc_topology_init(&topology);
        if (err) {
            throw Exception("Profile: unable to initialize hwloc", GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }

        err = hwloc_topology_load(topology);
        if (err) {
            throw Exception("Profile: unable to load topology in hwloc", GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }

        set = hwloc_bitmap_alloc();

        err = hwloc_get_cpubind(topology, set, HWLOC_CPUBIND_PROCESS);
        if (err) {
            throw Exception("Profile: unable to get process binding from hwloc", GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }

        hwloc_bitmap_foreach_begin(i, set) {
            m_cpu_list.push_front(i);
        }
        hwloc_bitmap_foreach_end();

        hwloc_bitmap_free(set);
        hwloc_topology_destroy(topology);
    }

    const struct geopm_sample_message_s GEOPM_INVALID_SAMPLE = {-1, 0, -1.0, -1.0, -1.0, -1.0};

    ProfileSampler::ProfileSampler(const std::string shm_key_base, size_t table_size, MPI_Comm comm)
        : m_ctl_shmem(shm_key_base, table_size)
        , m_ctl_msg((struct geopm_ctl_message_s *)m_ctl_shmem.pointer())
        , m_comm(comm)
        , m_num_rank(0)
    {
        std::string shm_key;

        while (m_ctl_msg->app_status != GEOPM_STATUS_INITIALIZED) {}

        std::set<int> rank_set;
        for (int i = 0; i < GEOPM_CONST_MAX_NUM_CPU; i++) {
            if (m_ctl_msg->cpu_rank[i] >= 0) {
                (void) rank_set.insert(m_ctl_msg->cpu_rank[i]);
            }
        }

        for (auto it = rank_set.begin(); it != rank_set.end(); ++it) {
            shm_key.assign(shm_key_base + "_" + std::to_string(*it));
            m_table_shmem.push_front(SharedMemory(shm_key, table_size));
            m_table.push_front(LockingHashTable<struct geopm_sample_message_s>(table_size, m_table_shmem.front().pointer()));
            m_region_entry_data.push_back(GEOPM_INVALID_SAMPLE);
        }

        m_num_rank = rank_set.size();
        m_elapsed_data.resize(m_num_rank);

        m_ctl_msg->ctl_status = GEOPM_STATUS_INITIALIZED;
    }

    ProfileSampler::~ProfileSampler(void)
    {

    }

    size_t ProfileSampler::capacity(void)
    {
        size_t cap = 0;

        for (auto it = m_table.begin(); it != m_table.end(); ++it) {
            cap += (*it).capacity();
        }

        return cap;
    }

    void ProfileSampler::sample(std::vector<std::pair<uint64_t, struct geopm_sample_message_s> > &contents, size_t &length)
    {
        size_t sub_length = 0;

        switch (m_ctl_msg->app_status) {
            case GEOPM_STATUS_ACTIVE:
                length = 0;
                for (auto it = m_table.begin(); it != m_table.end(); ++it) {
                    (*it).dump(contents.begin() + length, sub_length);
                    length += sub_length;
                }
                calculate_elapsed(contents, length);
                break;
            case GEOPM_STATUS_REPORT:
                report();
                break;
            case GEOPM_STATUS_SHUTDOWN:
                break;
            default:
                throw Exception("ProfileSampler: inavlid application status: " + std::to_string(m_ctl_msg->app_status), GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
    }

    void ProfileSampler::calculate_elapsed(const std::vector<std::pair<uint64_t, struct geopm_sample_message_s> > &contents, const size_t &length) {
        
    }

    bool ProfileSampler::do_shutdown(void)
    {
        return (m_ctl_msg->app_status == GEOPM_STATUS_SHUTDOWN);
    }

    void ProfileSampler::report(void)
    {
        int all_done = 0;
        int is_done = 0;
        size_t header_offset = 0;
        std::string file_name;
        std::string prof_name;
        std::set<std::string> name_set;

        m_ctl_msg->ctl_status = GEOPM_STATUS_REPORT;
        while (m_ctl_msg->app_status != GEOPM_STATUS_READY ||
               m_ctl_msg->app_status == GEOPM_STATUS_SHUTDOWN) {}
        if (m_ctl_msg->app_status != GEOPM_STATUS_SHUTDOWN) {
            file_name.assign((char *)m_table_shmem.front().pointer());
            header_offset += file_name.length() + 1;
            prof_name.assign((char *)m_table_shmem.front().pointer() + header_offset);
            header_offset += prof_name.length() + 1;
        }

        while (!all_done) {
            all_done = 1;
            for (auto it = m_table.begin(); it != m_table.end(); ++it) {
                is_done = (*it).name_set(header_offset, name_set);
                if (!is_done) {
                    all_done = 0;
                }
            }
            m_ctl_msg->ctl_status = GEOPM_STATUS_READY;
            while (m_ctl_msg->app_status != GEOPM_STATUS_READY &&
                   m_ctl_msg->app_status != GEOPM_STATUS_SHUTDOWN) {}
            if (!all_done && m_ctl_msg->app_status == GEOPM_STATUS_SHUTDOWN) {
                throw Exception("ProfileSampler::report(): Application shutdown while report was being generated", GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
            }
            header_offset = 0;
        }
    }
}

