#ifndef FILE_PATH_H
#define FILE_PATH_H

#include <string>
#include <algorithm>

/// Represents a path in the file system.
class FilePath {
public:
    FilePath(const std::string& path)
        : path_(path)
    {
        std::replace(path_.begin(), path_.end(), '\\', '/');
        auto pos = path_.rfind('/');
        base_ = (pos != std::string::npos) ? path_.substr(0, pos)  : ".";
        file_ = (pos != std::string::npos) ? path_.substr(pos + 1) : path_;
    }

    const std::string& path() const { return path_; }
    const std::string& base_name() const { return base_; }
    const std::string& file_name() const { return file_; }

    std::string extension() const {
        auto pos = file_.rfind('.');
        return (pos != std::string::npos) ? file_.substr(pos + 1) : std::string();
    }

    std::string remove_extension() const {
        auto pos = file_.rfind('.');
        return (pos != std::string::npos) ? file_.substr(0, pos) : file_;
    }

    operator const std::string& () const {
        return path();
    }

private:
    std::string path_;
    std::string base_;
    std::string file_;
};

#endif // FILE_PATH_H
