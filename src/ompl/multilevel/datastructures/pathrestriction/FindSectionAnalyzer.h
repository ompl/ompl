#pragma once
#include <ompl/multilevel/datastructures/pathrestriction/BasePathHead.h>

namespace ompl
{
    namespace multilevel
    {
        OMPL_CLASS_FORWARD(BasePathHead);
        class FindSectionAnalyzer
        {
          public:
            FindSectionAnalyzer(BasePathHeadPtr& head)
            {
                head_ = head;
            }
            using OccurenceMap = std::map<std::string, int>;

            void operator ()(std::string s)
            {
                if(enabled_)
                {
                    auto entry = map_.find(s);
                    if(entry == map_.end())
                    {
                        map_[s] = 1;
                    }else{
                        map_[s] = map_[s] + 1;
                    }
                    samples_++;
                }
            }

            void disable()
            {
                enabled_ = false;
            }

            void clear()
            {
              map_.clear();
            }
            void print()
            {
                if(enabled_)
                {
                    OccurenceMap::iterator itr; 
                    std::cout << std::string(80, '-') << std::endl;
                    std::cout << "FindSectionAnalyzer ("
                      << samples_ << " samples, location "
                      << head_->getLocationOnBasePath() << ")" << std::endl;
                    for (itr = map_.begin(); itr != map_.end(); ++itr) 
                    { 
                      std::cout << " > " << itr->first
                       << ": " << itr->second << std::endl;
                    } 
                    std::cout << std::string(80, '-') << std::endl;
                }
            }
          private:
            OccurenceMap map_;
            int samples_{0};
            BasePathHeadPtr head_;

            bool enabled_{true};
        };
    }
}

