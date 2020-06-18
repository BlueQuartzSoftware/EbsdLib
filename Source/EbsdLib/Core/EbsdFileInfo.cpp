

#include "EbsdFileInfo.h"

#define FILE_INFO_CLASS_NAME EbsdFileInfo

#include "EbsdFileSystemPath.cpp"

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
uint64_t FILE_INFO_CLASS_NAME::fileSize(const std::string& path)
{
  int error;
  MXA_STATBUF st;
  error = MXA_STAT(path.c_str(), &st);
  if (!error && (st.st_mode & S_IFMT) == S_IFREG)
  {
    return (uint64_t)(st.st_size);
  }
  return 0;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::string FILE_INFO_CLASS_NAME::extension(const std::string& fsPath)
{
  std::string::size_type pos = fsPath.find_last_of('.');
  // No '.' characters appeared in the path at all
  if(std::string::npos == pos)
  {
    return std::string();
  }
  // Look for a "Hidden" file  .som ./.some ../.something.something
  if(pos == 0 || fsPath[pos - 1] == FILE_INFO_CLASS_NAME::Separator)
  {
    return std::string();  // Return empty string, there is no extension
  }

  std::string::size_type slashpos = fsPath.find_last_of(FILE_INFO_CLASS_NAME::Separator);
  // Check for just a plain filename, ie a path with NO directory delimiters in the string
  if (std::string::npos == slashpos && std::string::npos != pos)
  {
    return fsPath.substr(pos + 1);
  }

  if (pos > slashpos)
  {
    return fsPath.substr(pos + 1);
  }

  return std::string();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::string FILE_INFO_CLASS_NAME::filename(const std::string& fsPath)
{

  std::string::size_type slashPos = fsPath.find_last_of(FILE_INFO_CLASS_NAME::Separator);
  if (slashPos == fsPath.size() - 1)
  {
    return FILE_INFO_CLASS_NAME::filename(fsPath.substr(0, fsPath.size() - 1));
  }

  std::string fn = fsPath.substr(slashPos + 1, fsPath.size() - slashPos);
  if (fn.at(fn.size()-1) == '.')
  {
    return fn.substr(0, fn.size() - 1);
  }
  return fn;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::string FILE_INFO_CLASS_NAME::baseName(const std::string& fsPath)
{
  std::string fname = FILE_INFO_CLASS_NAME::filename(fsPath);
  std::string ext = FILE_INFO_CLASS_NAME::extension(fsPath);
  std::string::size_type pos = fname.find_last_of(ext);
  if (pos != std::string::npos)
  {
    fname = fname.substr(0, fname.size() - ext.size() - 1);
  }

  //  std::string parentPath = FILE_INFO_CLASS_NAME::parentPath(fsPath);
  //  if (parentPath.size() > 0)
  //  {
  //    return parentPath + FILE_INFO_CLASS_NAME::getSeparator() + fname;
  //  }
  //  else
  //  {
  return fname;
  //  }
}
