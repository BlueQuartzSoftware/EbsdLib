
//
//  Copyright (c) 2009, Michael A. Jackson. BlueQuartz Software
//  All rights reserved.
//  BSD License: http://www.opensource.org/licenses/bsd-license.html
//
//

/* Note that some of this implementation was inspired by the Qt4 source code,
 * in particular the QDir and QFileEngine source codes. The idea is to keep the
 * API the same between my implementation and the Qt Implementation so that
 * switching between then is easy.
 *
 */
#include <ctype.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring> // Needed for memset
#include <iostream>
#include <vector>

#if defined (_MSC_VER)
#include <direct.h>
#define UNLINK _unlink
#define MXA_PATH_MAX MAX_PATH
#define MXA_GET_CWD _getcwd
#else
#define UNLINK ::unlink
#include <dirent.h>
#define MXA_PATH_MAX PATH_MAX
#define MXA_GET_CWD ::getcwd
#endif

#include <sys/stat.h>

#if defined (_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#define MXA_STATBUF    struct _stati64   // non-ANSI defs
#define MXA_STATBUF4TSTAT  struct _stati64   // non-ANSI defs
#define MXA_STAT     _stati64
#define MXA_FSTAT    _fstati64

#define MXA_STAT_REG   _S_IFREG
#define MXA_STAT_DIR   _S_IFDIR
#define MXA_STAT_MASK    _S_IFMT
#if defined(_S_IFLNK)
#  define MXA_STAT_LNK   _S_IFLNK
#endif

#elif defined (__APPLE__)

#define MXA_STATBUF    struct stat
#define MXA_STATBUF4TSTAT  struct stat
#define MXA_STAT     stat
#define MXA_FSTAT    fstat

#define MXA_STAT_REG   S_IFREG
#define MXA_STAT_DIR   S_IFDIR
#define MXA_STAT_MASK    S_IFMT
#define MXA_STAT_LNK   S_IFLNK

#else
#define MXA_STATBUF    struct stat
#define MXA_STATBUF4TSTAT  struct stat
#define MXA_STAT     stat
#define MXA_FSTAT    fstat

#define MXA_STAT_REG   S_IFREG
#define MXA_STAT_DIR   S_IFDIR
#define MXA_STAT_MASK    S_IFMT
#define MXA_STAT_LNK   S_IFLNK
#endif


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
FILE_INFO_CLASS_NAME::FILE_INFO_CLASS_NAME() = default;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
FILE_INFO_CLASS_NAME::~FILE_INFO_CLASS_NAME() = default;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::string FILE_INFO_CLASS_NAME::getSeparator()
{
#if defined (WIN32)
      return "\\";
#else
      return "/";
#endif
}


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
bool FILE_INFO_CLASS_NAME::isDirectory(const std::string& fsPath)
{
#if defined (WIN32)
  bool existed = false;
  return FILE_INFO_CLASS_NAME::isDirPath(fsPath, &existed);
#else
  int error;
  MXA_STATBUF st;
  error = MXA_STAT(fsPath.c_str(), &st);
  if (!error && (st.st_mode & S_IFMT) == S_IFDIR)
  {
    return true;
  }
  return false;
#endif
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
bool FILE_INFO_CLASS_NAME::isFile(const std::string& fsPath)
{
  int error;
  MXA_STATBUF st;
  error = MXA_STAT(fsPath.c_str(), &st);
  if (!error && (st.st_mode & S_IFMT) == S_IFREG)
  {
    return true;
  }
  return false;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
bool FILE_INFO_CLASS_NAME::isRelativePath(const std::string& path)
{
#if defined (WIN32)
  if (path.length() > 2 && isalpha(path[0]) == false && path[1] != ':' && path[2] != '\\') {return true;}
#else
  if (path.length() > 0 && path[0] != '/') { return true; }
#endif
  return false;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
bool FILE_INFO_CLASS_NAME::isAbsolutePath(const std::string& path)
{
#if defined (WIN32)
  if (path.length() > 2 && isalpha(path[0]) != 0 && path[1] == ':' && path[2] == '\\') {return true;}
  if (path.length() > 1 && path[0] == '\\' && path[1] == '\\') { return true;}
#else
  if (path.length() > 0 && path[0] == '/') { return true; }
#endif
  return false;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::string FILE_INFO_CLASS_NAME::currentPath()
{
  std::string currentPath;
  MXA_STATBUF st;
  if (0 == MXA_STAT(".", &st) )
  {
    char currentName[MXA_PATH_MAX+1];
    char* result = NULL;
    ::memset(&currentName[0], 0, MXA_PATH_MAX + 1); // Clear everything to zeros.
    result = MXA_GET_CWD(currentName, MXA_PATH_MAX);
    if (NULL == result)
    {
      std::cout << "Error: FILE_INFO_CLASS_NAME::currentPath result was NULL." << std::endl;
    }
    else
    {
      currentPath = std::string(currentName);
    }

  }
  else
  {
    std::cout << "Error: FILE_INFO_CLASS_NAME::currentPath stat function failed." << std::endl;
  }
#if defined(WIN32)
  currentPath = FILE_INFO_CLASS_NAME::cleanPath(currentPath);
#endif

  return currentPath;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::string FILE_INFO_CLASS_NAME::absolutePath(const std::string& path)
{
  std::string abspath = FILE_INFO_CLASS_NAME::toNativeSeparators(path);
  if(true == FILE_INFO_CLASS_NAME::isAbsolutePath(abspath))
  { return path; }

  abspath = FILE_INFO_CLASS_NAME::currentPath();
  if(abspath[abspath.length() - 1] != FILE_INFO_CLASS_NAME::Separator)
  {
    abspath = abspath + FILE_INFO_CLASS_NAME::Separator;
  }
  abspath.append(path);
  abspath = FILE_INFO_CLASS_NAME::cleanPath(abspath);
  return abspath;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::string FILE_INFO_CLASS_NAME::parentPath(const std::string& path)
{
  std::string curAbsPath = FILE_INFO_CLASS_NAME::absolutePath(path);
  curAbsPath = FILE_INFO_CLASS_NAME::fromNativeSeparators(curAbsPath);
  std::string::size_type nextToLastSlashPos = 0;
  std::string::size_type lastSlashPos = curAbsPath.find_last_of(FILE_INFO_CLASS_NAME::UnixSeparator);
  // Remove trailing '/' if found
  if (lastSlashPos == curAbsPath.length() - 1)
  {
    curAbsPath = curAbsPath.substr(0, curAbsPath.length()-2);
    lastSlashPos = curAbsPath.find_last_of(FILE_INFO_CLASS_NAME::Separator);
  }

  if (lastSlashPos > 0)
  {
    nextToLastSlashPos = curAbsPath.find_last_of(FILE_INFO_CLASS_NAME::UnixSeparator, lastSlashPos - 1);
  }

  if (nextToLastSlashPos == std::string::npos) // Only 1 slash found, return the root directory
  {
#if defined (WIN32)
    curAbsPath = curAbsPath.substr(0, 3);
    return FILE_INFO_CLASS_NAME::toNativeSeparators(curAbsPath);
#else
    return curAbsPath.substr(0, 1);
#endif
  }

  curAbsPath = curAbsPath.substr(0, lastSlashPos);
#if defined (WIN32)
  curAbsPath = FILE_INFO_CLASS_NAME::toNativeSeparators(curAbsPath);
#endif
  return curAbsPath;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
bool FILE_INFO_CLASS_NAME::exists(const std::string& fsPath)
{
  int error;
  std::string dirName(fsPath);
  // Both windows and OS X both don't like trailing slashes so just get rid of them
  // for all Operating Systems.
  if(dirName[dirName.length() - 1] == FILE_INFO_CLASS_NAME::Separator)
  {
    dirName = dirName.substr(0, dirName.length() - 1);
  }
  MXA_STATBUF st;
  error = MXA_STAT(dirName.c_str(), &st);
  return (error == 0);
}




#if defined (WIN32)
// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
bool FILE_INFO_CLASS_NAME::isDirPath(const std::string& folderPath, bool* existed)
{
    std::string fsPath = folderPath;
    if(fsPath.length() == 2 && fsPath.at(1) == ':')
      fsPath += FILE_INFO_CLASS_NAME::Separator;

    DWORD fileAttrib = INVALID_FILE_ATTRIBUTES;
    fileAttrib = ::GetFileAttributesA(fsPath.c_str() );

    if (existed)
        *existed = fileAttrib != INVALID_FILE_ATTRIBUTES;

    if (fileAttrib == INVALID_FILE_ATTRIBUTES)
        return false;

    return (fileAttrib & FILE_ATTRIBUTE_DIRECTORY) ? true : false;
}
#endif


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::string FILE_INFO_CLASS_NAME::fromNativeSeparators(const std::string& fsPath)
{
  std::string path(fsPath);
#if defined (WIN32)
  for(int i = 0; i < (int)path.length(); i++)
  {
    if(path[i] == FILE_INFO_CLASS_NAME::Separator)
      path[i] = FILE_INFO_CLASS_NAME::UnixSeparator;
  }
#endif
  return path;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::string FILE_INFO_CLASS_NAME::toNativeSeparators(const std::string& fsPath)
{
    std::string path(fsPath);
#if defined (WIN32)
    for(int i = 0; i < (int)path.length(); i++)
    {
      if(path[i] == FILE_INFO_CLASS_NAME::UnixSeparator)
        path[i] = FILE_INFO_CLASS_NAME::Separator;
    }
#endif
    return path;
}


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
std::string FILE_INFO_CLASS_NAME::cleanPath(const std::string& fsPath)
{
    if (fsPath.length() == 0)
        return fsPath;
      std::string path(fsPath);
     char slash = '/';
     char dot = '.';
     if(FILE_INFO_CLASS_NAME::Separator != FILE_INFO_CLASS_NAME::UnixSeparator)
     {
       path = fromNativeSeparators(path);
     }

     // Peel off any trailing slash
     if (path[path.length() -1 ] == slash)
     {
       path = path.substr(0, path.length() -1);
     }

     std::vector<std::string> stk;
     std::string::size_type pos = 0;
     std::string::size_type pos1 = 0;

     pos = path.find(slash, pos);
     pos1 = path.find(slash, pos + 1);
   #if defined (WIN32)
     // Check for UNC style paths first
     if (pos == 0 && pos1 == 1)
     {
       pos1 = path.find(slash, pos1 + 1);
     } else
   #endif
     if (pos != 0)
     {
       stk.push_back(path.substr(0, pos));
     }
     // check for a top level Unix Path:
     if (pos == 0 && pos1 == std::string::npos)
     {
         stk.push_back(path);
     }


     while (pos1 != std::string::npos)
     {
       if (pos1 - pos == 3 && path[pos+1] == dot && path[pos+2] == dot)
       {
       //  std::cout << "Popping back element" << std::endl;
         if (stk.size() > 0) {
           stk.pop_back();
         }
       }
       else if (pos1 - pos == 2 && path[pos+1] == dot )
       {

       }
       else if (pos + 1 == pos1) {

       }
       else {
         stk.push_back(path.substr(pos, pos1-pos));
       }
       pos = pos1;
       pos1 = path.find(slash, pos + 1);
       if (pos1 == std::string::npos)
       {
         stk.push_back(path.substr(pos, path.length() - pos));
       }
     }
     std::string ret;
     for (std::vector<std::string>::iterator iter = stk.begin(); iter != stk.end(); ++iter ) {
       ret.append(*iter);
     }
     ret = toNativeSeparators(ret);
     #if defined (WIN32)
     if (ret.length() > 2
       && isalpha(ret[0]) != 0
       && islower(ret[0]) != 0
        && ret[1] == ':' && ret[2] == '\\')
      {
        //we have a lower case drive letter which needs to be changed to upper case.
        ret[0] = toupper(ret[0]);
      }
#endif
     return ret;
}
