#include <dirent.h>

bool FileExists(const std::string &filename) {
  std::ifstream file(filename);
  return (!file.fail());
}

void GetFilesInDirectory(const std::string &directory, std::vector<std::string> &file_list, const std::string &search_string) {
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir (directory.c_str())) != NULL) {
        while ((ent = readdir (dir)) != NULL) {
            std::string filename(ent->d_name);
            if (filename.find(search_string) != std::string::npos && filename != "." && filename != "..")
                file_list.push_back(filename);
        }
        closedir (dir);
    } else
        perror ("Error: could not look into directory!");
}
