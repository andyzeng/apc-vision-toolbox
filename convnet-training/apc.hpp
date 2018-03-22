// ---------------------------------------------------------
// Copyright (c) 2016, Andy Zeng
// 
// This file is part of the APC Vision Toolbox and is available 
// under the terms of the Simplified BSD License provided in 
// LICENSE. Please retain this notice and LICENSE if you use 
// this file (or any portion of it) in your project.
// ---------------------------------------------------------

#include "system_utils.h"
#include "depth_utils.h"
#include "random_utils.h"

template <class T>
class APCDataLayer : public DataLayer {
    std::future<void> lock;

    std::vector<FILE*> dataFILE;
    std::vector<StorageT*> dataCPU;
    std::vector<StorageT*> dataGPU;
    std::vector<StorageT*> labelCPU;
    std::vector<StorageT*> labelGPU;

    int epoch_prefetch;
public:
    std::vector<std::string> file_data;

    int batch_size;
    int curr_obj_idx = 0;
    std::vector<std::string> object_list;
    int num_objects;

    int numofitems() { return 0; };

    void init() {
        epoch_prefetch  = 0;
        train_me = true;
        std::cout << "APCDataLayer: " << std::endl;
        dataCPU.resize(2);
        dataGPU.resize(2);
        labelCPU.resize(1);
        labelGPU.resize(1);
        dataFILE.resize(file_data.size());

        // List objects found under data directory
        std::cout << "    Loading data from directory: " << file_data[0] << std::endl;
        GetFilesInDirectory(file_data[0], object_list, "");
        num_objects = object_list.size();
        std::sort(object_list.begin(), object_list.end());
        for (int i = 0 ; i < num_objects; i++)
            std::cout << "        " << object_list[i] << std::endl;

        // Compute batch data sizes
        std::vector<int> image_dim;
        image_dim.push_back(batch_size); image_dim.push_back(3); image_dim.push_back(480); image_dim.push_back(640);
        dataCPU[0]  = new StorageT[numel(image_dim)];
        dataCPU[1]  = new StorageT[numel(image_dim)];

        // Compute batch label sizes
        std::vector<int> label_dim;
        label_dim.push_back(batch_size); label_dim.push_back(1); label_dim.push_back(480); label_dim.push_back(640);
        labelCPU[0] = new StorageT[numel(label_dim)];
    };

    APCDataLayer(std::string name_, Phase phase_, std::vector<std::string> file_data_, int batch_size_):
        DataLayer(name_), file_data(file_data_), batch_size(batch_size_) {
        phase = phase_;
        init();
    };

    APCDataLayer(JSON* json) {
        SetOrDie(json, name)
        SetValue(json, phase,       Training)
        SetOrDie(json, file_data    )
        SetOrDie(json, batch_size   )
        init();
    };

    ~APCDataLayer() {
        if (lock.valid()) lock.wait();
        for (int i = 0; i < dataFILE.size(); ++i)
            if (dataFILE[i] != NULL) fclose(dataFILE[i]);
        for (int i = 0; i < dataCPU.size(); ++i)
            if (dataCPU[i] != NULL) delete [] dataCPU[i];
        for (int i = 0; i < labelCPU.size(); ++i)
            if (labelCPU[i] != NULL) delete [] labelCPU[i];
        for (int i = 0; i < dataGPU.size(); ++i)
            if (dataGPU[i] != NULL) checkCUDA(__LINE__, cudaFree(dataGPU[i]));
        for (int i = 0; i < labelGPU.size(); ++i)
            if (labelGPU[i] != NULL) checkCUDA(__LINE__, cudaFree(labelGPU[i]));
    };

    void shuffle() {};

    void prefetch() {

        checkCUDA(__LINE__, cudaSetDevice(GPU));

        std::string data_directory = file_data[0];

        for (int batch_idx = 0; batch_idx < batch_size; batch_idx++) {
            std::string curr_obj_name = object_list[curr_obj_idx];
            std::string curr_obj_directory = data_directory + "/" + curr_obj_name;

            // Select a random sequence from object directory
            std::vector<std::string> sequence_list;
            GetFilesInDirectory(curr_obj_directory, sequence_list, "scene-0");
            std::sort(sequence_list.begin(), sequence_list.end());
            int rand_sequence_idx = (int)floor(GetRandomFloat(0, (float)sequence_list.size()));
            std::string curr_sequence_name = sequence_list[rand_sequence_idx];
            std::string curr_sequence_directory = curr_obj_directory + "/" + curr_sequence_name;

            // Select a random image from the sequence
            std::vector<std::string> image_list;
            GetFilesInDirectory(curr_sequence_directory, image_list, ".color.png");
            std::sort(image_list.begin(), image_list.end());
            int rand_image_idx = (int)floor(GetRandomFloat(0, (float)image_list.size()));
            std::string curr_image_name = image_list[rand_image_idx];

            // Debug
            // std::cout << curr_sequence_directory + "/" + curr_image_name << std::endl;

            // Read color RGB data (BGR, mean subtracted)
            std::string curr_RGB_file = curr_sequence_directory + "/" + curr_image_name;
            cv::Mat curr_RGB_image = cv::imread(curr_RGB_file.c_str(), CV_LOAD_IMAGE_COLOR);
            uint8_t * curr_RGB_raw = curr_RGB_image.data;
            StorageT * curr_RGB_data = new StorageT[3 * 480 * 640];
            for (int tmp_row = 0; tmp_row < 480; tmp_row++)
                for (int tmp_col = 0; tmp_col < 640; tmp_col++) {
                    // curr_RGB_data[0 * 480 * 640 + tmp_row * 640 + tmp_col] = CPUCompute2StorageT(((float) curr_RGB_image.at<cv::Vec3b>(tmp_row, tmp_col)[0]) - 102.9801f); //102.9801f; // B
                    // curr_RGB_data[1 * 480 * 640 + tmp_row * 640 + tmp_col] = CPUCompute2StorageT(((float) curr_RGB_image.at<cv::Vec3b>(tmp_row, tmp_col)[1]) - 115.9465f); //115.9465f; // G
                    // curr_RGB_data[2 * 480 * 640 + tmp_row * 640 + tmp_col] = CPUCompute2StorageT(((float) curr_RGB_image.at<cv::Vec3b>(tmp_row, tmp_col)[2]) - 122.7717f); //122.7717f; // R
                    curr_RGB_data[0 * 480 * 640 + tmp_row * 640 + tmp_col] = CPUCompute2StorageT(ComputeT(curr_RGB_raw[0 + 3 * (tmp_col + 640 * tmp_row)]) - ComputeT(102.9801f)); // B
                    curr_RGB_data[1 * 480 * 640 + tmp_row * 640 + tmp_col] = CPUCompute2StorageT(ComputeT(curr_RGB_raw[1 + 3 * (tmp_col + 640 * tmp_row)]) - ComputeT(115.9465f)); // G
                    curr_RGB_data[2 * 480 * 640 + tmp_row * 640 + tmp_col] = CPUCompute2StorageT(ComputeT(curr_RGB_raw[2 + 3 * (tmp_col + 640 * tmp_row)]) - ComputeT(122.7717f)); // R
                }

            // Read depth HHA data (BGR, mean subtracted)
            std::string curr_HHA_file = curr_sequence_directory + "/HHA/" + curr_image_name.substr(0, curr_image_name.length() - 10) + ".HHA.png";
            cv::Mat curr_HHA_image = cv::imread(curr_HHA_file.c_str(), CV_LOAD_IMAGE_COLOR);
            uint8_t * curr_HHA_raw = curr_HHA_image.data;
            StorageT * curr_HHA_data = new StorageT[3 * 480 * 640];
            for (int tmp_row = 0; tmp_row < 480; tmp_row++)
                for (int tmp_col = 0; tmp_col < 640; tmp_col++) {
                    // curr_HHA_data[0 * 480 * 640 + tmp_row * 640 + tmp_col] = CPUCompute2StorageT(((float) curr_HHA_image.at<cv::Vec3b>(tmp_row, tmp_col)[0]) - 102.9801f); //102.9801f; // B
                    // curr_HHA_data[1 * 480 * 640 + tmp_row * 640 + tmp_col] = CPUCompute2StorageT(((float) curr_HHA_image.at<cv::Vec3b>(tmp_row, tmp_col)[1]) - 115.9465f); //115.9465f; // G
                    // curr_HHA_data[2 * 480 * 640 + tmp_row * 640 + tmp_col] = CPUCompute2StorageT(((float) curr_HHA_image.at<cv::Vec3b>(tmp_row, tmp_col)[2]) - 122.7717f); //122.7717f; // R
                    curr_HHA_data[0 * 480 * 640 + tmp_row * 640 + tmp_col] = CPUCompute2StorageT(ComputeT(curr_HHA_raw[0 + 3 * (tmp_col + 640 * tmp_row)]) - ComputeT(102.9801f)); // B
                    curr_HHA_data[1 * 480 * 640 + tmp_row * 640 + tmp_col] = CPUCompute2StorageT(ComputeT(curr_HHA_raw[1 + 3 * (tmp_col + 640 * tmp_row)]) - ComputeT(115.9465f)); // G
                    curr_HHA_data[2 * 480 * 640 + tmp_row * 640 + tmp_col] = CPUCompute2StorageT(ComputeT(curr_HHA_raw[2 + 3 * (tmp_col + 640 * tmp_row)]) - ComputeT(122.7717f)); // R
                }

            // Debug
            // FILE * fp = fopen(("test" + std::to_string(batch_idx) + ".color.bin").c_str(), "wb");
            // fwrite(curr_RGB_data, sizeof(float), 3 * 480 * 640, fp);
            // fclose(fp);

            // Read label masks
            std::string curr_mask_file = curr_sequence_directory + "/masks/" + curr_image_name.substr(0, curr_image_name.length() - 10) + ".mask.png";
            cv::Mat curr_mask = cv::imread(curr_mask_file.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
            StorageT * curr_label_data = new StorageT[480 * 640];
            for (int tmp_row = 0; tmp_row < 480; tmp_row++)
                for (int tmp_col = 0; tmp_col < 640; tmp_col++) {
                    if (((int) curr_mask.at<unsigned char>(tmp_row, tmp_col)) > 0)
                        curr_label_data[tmp_row * 640 + tmp_col] = CPUCompute2StorageT(curr_obj_idx + 1); // Give object label 
                    else
                        curr_label_data[tmp_row * 640 + tmp_col] = CPUCompute2StorageT(0); // Background
                }

            // Debug
            // fp = fopen(("test" + std::to_string(batch_idx) + ".mask.bin").c_str(), "wb");
            // fwrite(curr_label_data, sizeof(float), 480 * 640, fp);
            // fclose(fp);

            // Copy data to GPU
            checkCUDA(__LINE__, cudaMemcpy(&(dataGPU[0][batch_idx * 3 * 480 * 640]), curr_RGB_data, 3 * 480 * 640 * sizeofStorageT, cudaMemcpyHostToDevice));
            checkCUDA(__LINE__, cudaMemcpy(&(dataGPU[1][batch_idx * 3 * 480 * 640]), curr_HHA_data, 3 * 480 * 640 * sizeofStorageT, cudaMemcpyHostToDevice));
            checkCUDA(__LINE__, cudaMemcpy(&(labelGPU[0][batch_idx * 480 * 640]), curr_label_data, 480 * 640 * sizeofStorageT, cudaMemcpyHostToDevice));

            // Clear memory
            delete [] curr_RGB_data;
            delete [] curr_HHA_data;
            delete [] curr_label_data;

            // Iterate through object list
            curr_obj_idx = curr_obj_idx + 1;
            if (curr_obj_idx >= num_objects)
                curr_obj_idx = 0;
        }

        // Debug
        // float * image_data = new float[batch_size * 3 * 480 * 640];
        // float * label_data = new float[batch_size * 1 * 480 * 640];
        // checkCUDA(__LINE__, cudaMemcpy(image_data, dataGPU[0], batch_size * 3 * 480 * 640 * sizeof(float), cudaMemcpyDeviceToHost));
        // checkCUDA(__LINE__, cudaMemcpy(label_data, labelGPU[0], batch_size * 480 * 640 * sizeof(float), cudaMemcpyDeviceToHost));
        // for (int i = 0; i < batch_size * 3 * 480 * 640; i++)
        //     std::cout << image_data[i] << std::endl;
        // for (int i = 0; i < batch_size * 480 * 640; i++)
        //     std::cout << label_data[i] << std::endl;
    };

    void forward(Phase phase_) {
        lock.wait();
        epoch = epoch_prefetch;
        std::swap(out[0]->dataGPU, dataGPU[0]);
        std::swap(out[1]->dataGPU, dataGPU[1]);
        std::swap(out[2]->dataGPU, labelGPU[0]);
        lock = std::async(std::launch::async, &APCDataLayer<T>::prefetch, this);
    };


    size_t Malloc(Phase phase_) {
        if (phase == Training && phase_ == Testing) return 0;
        if (out.size() != 3) {
            std::cout << "APCDataLayer: incorrect # of out's" << std::endl;
            FatalError(__LINE__);
        }
        size_t memoryBytes = 0;
        std::cout << (train_me ? "* " : "  ");
        std::cout << name << std::endl;

        // CPU/GPU malloc data
        std::vector<int> image_dim;
        image_dim.push_back(batch_size); image_dim.push_back(3); image_dim.push_back(480); image_dim.push_back(640);
        out[0]->need_diff = false;
        out[0]->receptive_field.resize(image_dim.size() - 2); fill_n(out[0]->receptive_field.begin(), image_dim.size() - 2, 1);
        out[0]->receptive_gap.resize(image_dim.size() - 2);   fill_n(out[0]->receptive_gap.begin(), image_dim.size() - 2, 1);
        out[0]->receptive_offset.resize(image_dim.size() - 2); fill_n(out[0]->receptive_offset.begin(), image_dim.size() - 2, 0);
        memoryBytes += out[0]->Malloc(image_dim);
        checkCUDA(__LINE__, cudaMalloc(&dataGPU[0], numel(image_dim) * sizeofStorageT) );
        memoryBytes += numel(image_dim) * sizeofStorageT;
        out[1]->need_diff = false;
        out[1]->receptive_field.resize(image_dim.size() - 2); fill_n(out[1]->receptive_field.begin(), image_dim.size() - 2, 1);
        out[1]->receptive_gap.resize(image_dim.size() - 2);   fill_n(out[1]->receptive_gap.begin(), image_dim.size() - 2, 1);
        out[1]->receptive_offset.resize(image_dim.size() - 2); fill_n(out[1]->receptive_offset.begin(), image_dim.size() - 2, 0);
        memoryBytes += out[1]->Malloc(image_dim);
        checkCUDA(__LINE__, cudaMalloc(&dataGPU[1], numel(image_dim) * sizeofStorageT) );
        memoryBytes += numel(image_dim) * sizeofStorageT;

        // CPU/GPU malloc labels
        std::vector<int> label_dim;
        label_dim.push_back(batch_size); label_dim.push_back(1); label_dim.push_back(480); label_dim.push_back(640);
        out[2]->need_diff = false;
        memoryBytes += out[2]->Malloc(label_dim);
        checkCUDA(__LINE__, cudaMalloc(&labelGPU[0], numel(label_dim) * sizeofStorageT) );
        memoryBytes += numel(label_dim) * sizeofStorageT;

        lock = std::async(std::launch::async, &APCDataLayer<T>::prefetch, this);
        return memoryBytes;
    };
};
