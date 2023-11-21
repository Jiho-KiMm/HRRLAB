/*
 * SaveData.h
 *
 *  Created on: 2021. 3. 15.
 *      Author: KimKyungHwan
*/
#ifndef SAVEDATA_H_
#define SAVEDATA_H_
#include <iostream>
#include <fstream>
#include <string>

class SaveData
{
private:
    enum
    {
        N = 3,
        MAX_Data_NUM = 50000, //50s
        MAX_FILE_NUM = 30,

    };
    std::ofstream fout;
    char *fileName;
    std::string file_list[MAX_FILE_NUM];
    std::string default_dir;
    int file_num, file_index, data_num;
    bool init_file[MAX_FILE_NUM];
    double Data_array[MAX_FILE_NUM][MAX_Data_NUM][N];
    int data_index;
    void FileOpen(int);
    void FileClose(int);

public:
    SaveData(const std::string directory = "/home/kkh/SaveData/");
    SaveData(const std::string file[], int size, const std::string directory = "/home/kkh/SaveData/");
    virtual ~SaveData();
    void SetFileName(const std::string file[], int size);
    void Save(const double *data);
    void Save(double data1,double data2,double data3);
    void RtSave(const double *data);
    void FileOut();
    /*
 * ofstream fout(file name )
 * fout<<
 * fout.close
*/
};
SaveData::SaveData(const std::string directory)
{
    file_index = 0;
    data_num = 0;
    data_index = 0;
    default_dir = directory;
    for (int i = 0; i < MAX_FILE_NUM; ++i)
    {
        init_file[i] = false;
    }
}
SaveData::SaveData(const std::string file[], int size, const std::string directory)
{
    file_index = 0;
    data_num = 0;
    data_index = 0;
    default_dir = directory;
    for (int i = 0; i < MAX_FILE_NUM; ++i)
    {
        init_file[i] = false;
    }
    for (int i = 0; i < size; ++i)
    {
        file_list[i] = file[i];
    }
    file_num = size;
    //  FileOpen();
}
SaveData::~SaveData() {}
void SaveData::SetFileName(const std::string file[], int size)
{
    for (int i = 0; i < size; ++i)
    {
        file_list[i] = file[i];
    }
    file_num = size;
}
void SaveData::FileOpen(int i)
{
    std::string dir;
    dir = default_dir + file_list[i] + ".txt"; // "/home/kkh/SaveData/file.txt"
    fout.open(dir.c_str(), std::ios_base::out | std::ios_base::trunc);
    init_file[i] = true;
    if (!fout)
        std::cerr << "Error:" << file_list[i] << "open failed" << std::endl;
}
void SaveData::FileClose(int i)
{
    fout.close();
}
void SaveData::FileOut()
{
    for (int fi = 0; fi < file_num; ++fi)
    {
        FileOpen(fi);
        for (int di = 0; di < data_num; ++di)
        {
            fout << Data_array[fi][di][0] << " " << Data_array[fi][di][1] << " " << Data_array[fi][di][2] << "\n";
        }
        fout.close();
        std::cout << "\nSaved " << file_list[fi];
    }
}

void SaveData::Save(const double *data)
{
    for (int i = 0; i < N; ++i)
    {
        Data_array[file_index][data_num][i] = data[i];
    }

    //next file ready
    if (file_index < file_num - 1)
        file_index++;
    else
    {
        file_index = 0;
        if (data_num < MAX_Data_NUM - 1)
            data_num++;
    }
}
void SaveData::Save(double data1,double data2,double data3)
{
	double data[3]={data1,data2,data3};
    for (int i = 0; i < N; ++i)
    {
        Data_array[file_index][data_num][i] = data[i];
    }

    //next file ready
    if (file_index < file_num - 1)
        file_index++;
    else
    {
        file_index = 0;
        if (data_num < MAX_Data_NUM - 1)
            data_num++;
    }
}


//real time save
//void SaveData::RtSave(const double *data)
//{
//    if (data_num < MAX_Data_NUM)
//    {
//        std::string dir;
//        dir = default_dir + file_list[file_index] + ".txt"; // "/home/kkh/SaveData/file.txt"
//        if (init_file[file_index])
//            fout.open(dir, std::ios_base::out | std::ios_base::app);
//        else
//        {
//            fout.open(dir, std::ios_base::out | std::ios_base::trunc);
//            init_file[file_index] = true;
//        }
//
//        if (fout)
//        {
//            fout << data[0] << " " << data[1] << " " << data[2] << "\n";
//            fout.close();
//        }
//        else
//            std::cerr << "Error:" << file_list[file_index] << "open failed" << std::endl;
//
//        if (file_index < file_num - 1)
//            file_index++;
//        else
//        {
//            file_index = 0;
//            data_num++;
//        }
//    }
//}
#endif /* SAVEDATA_H_ */
