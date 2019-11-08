#include <log2plot/config_manager.h>

namespace log2plot
{

void ConfigManager::read(TagList tags, vpArray2D<double> &M,
                         const uint rows, const uint cols) const
{
  std::vector<std::vector<std::string>> content;
  // try to read as 2-dim
  uint found_rows(0), found_cols(0);
  try
  {
    read(tags, content);
    found_rows = static_cast<uint>(content.size());
  }
  catch(...)  {}

  if(found_rows)
  {
    found_cols = static_cast<uint>(content[0].size());
    // check dimension
    for(const auto &row: content)
    {
      if(row.size() != found_cols)
        throw std::runtime_error(tagPathMessage(tags, "log2plot::ConfigManager::read(vpArray2D): dimension error"));
    }
  }
  else
  {
    content.push_back(read<std::vector<std::string>>(tags));
    found_rows = 1;
    found_cols = static_cast<uint>(content[0].size());
  }

  // resize M to passed dimension
  if(cols != 0 && rows != 0)
    M.resize(rows, cols);

  if(M.getCols() == 0 && M.getRows() == 0)
    M.resize(found_rows, found_cols);

  // check for dim.0 column vector
  if(M.getCols() == 1 && M.getRows() == 0)
  {
    if(found_cols == 1)
      M.resize(found_rows, 1);
    else if(found_rows == 1) // will read as transpose of row vector
      M.resize(found_cols, 1);
  }

  // check for dim.0 row vector
  if(M.getCols() == 0 && M.getRows() == 1 && found_rows == 1)
    M.resize(1, found_cols);

  if(found_cols * found_rows != M.getCols() * M.getRows())
    throw std::runtime_error(tagPathMessage(tags, "log2plot::ConfigManager::read(vpArray2D): dimension error"));


  if(M.getRows() == found_rows)
  {
    // copy without transpose
    for(uint row = 0; row < M.getRows(); ++row)
    {
      for(uint col = 0; col < M.getCols(); ++col)
        M[row][col] = str2double(content[row][col]);
    }
  }
  else
  {
    // copy with transpose - for 1-dim row to column
    for(uint row = 0; row < M.getRows(); ++row)
      M[row][0] = str2double(content[0][row]);
  }
}

void ConfigManager::read(TagList tags, vpHomogeneousMatrix &M) const
{
  // try to read as 6-dim vector
  try
  {
    vpPoseVector p;
    read(tags, p);
    M.buildFrom(p);
  } catch (...)
  {
    // read as 4x4 matrix
    read(tags, static_cast<vpArray2D<double>&>(M));
  }
}

void ConfigManager::read(TagList tags, vpRotationMatrix &R) const
{
  // try to read as 3-dim vector
  try
  {
    vpThetaUVector tu;
    read(tags, tu);
    R.buildFrom(tu);
  } catch (...)
  {
    // read as 3x3 matrix
    read(tags, static_cast<vpArray2D<double>&>(R));
  }
}

}


