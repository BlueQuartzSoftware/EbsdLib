#pragma once

#include <algorithm>
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>

class GenerateFunctionList
{

public:
  GenerateFunctionList() = default;
  virtual ~GenerateFunctionList() = default;

  using EntryType = std::vector<int>;

  /**
   * @brief GenerateTable
   * @param n
   * @param k
   * @return
   */
  std::vector<EntryType> GeneratePermutationsOfCombinations(int n, int k)
  {
    m_Combinations.clear();
    m_Permutations.clear();

    combinations(n, k);
    // for(size_t i = 0; i < m_Combinations.size(); i++)
    for(auto& combination : m_Combinations)
    {
      permutation(static_cast<int>(combination.size()), combination);
    }
    return m_Permutations;
  }

protected:
  /**
   * @brief combinations
   * @param n
   * @param k
   */
  void combinations(int n, int k)
  {
    std::string bitmask(k, 1); // K leading 1's
    bitmask.resize(n, 0);      // N-K trailing 0's

    do
    {
      std::vector<int> entry;
      for(std::string::size_type i = 0; i < n; ++i) // [0..N-1] integers
      {
        if(bitmask[i])
        {
          entry.push_back(static_cast<int>(i));
        }
      }
      m_Combinations.push_back(entry);

    } while(std::prev_permutation(bitmask.begin(), bitmask.end()));
  }

  /**
   * @brief permutation
   * @param n
   * @param ch
   * @return
   */
  int permutation(int n, std::vector<int> ch)
  {
    int i, j;
    int temp;
    std::vector<int>::size_type N = ch.size();

    if(n == 0)
    {
      std::vector<int> entry;

      for(j = static_cast<int>(N - 1); j >= 0; j--)
      {
        entry.push_back(ch[j]);
        // std::cout<<ch[j];
      }
      //        std::cout<<std::endl;
      m_Permutations.push_back(entry);
      return 0;
    }
    for(i = 0; i < n; i++)
    {
      temp = ch[i];
      for(j = i + 1; j < n; j++)
      {
        ch[j - 1] = ch[j];
      }
      ch[n - 1] = temp;
      // shift
      permutation(n - 1, ch);
      for(j = n - 1; j > i; j--)
      {
        ch[j] = ch[j - 1];
      }
      ch[i] = temp;
      // and shift back agian
    }
    return 1;
  }

private:
  std::vector<EntryType> m_Combinations;
  std::vector<EntryType> m_Permutations;

public:
  GenerateFunctionList(const GenerateFunctionList&) = delete;            // Copy Constructor Not Implemented
  GenerateFunctionList(GenerateFunctionList&&) = delete;                 // Move Constructor Not Implemented
  GenerateFunctionList& operator=(const GenerateFunctionList&) = delete; // Copy Assignment Not Implemented
  GenerateFunctionList& operator=(GenerateFunctionList&&) = delete;      // Move Assignment Not Implemented
};
