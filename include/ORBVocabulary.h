/**
* This file is part of ORB-SLAM2.
* orb特征字典 创建特征的字典线性表示向量 加快匹配
*/


#ifndef ORBVOCABULARY_H
#define ORBVOCABULARY_H

#include"DBoW2/FORB.h"
#include"DBoW2/TemplatedVocabulary.h"

namespace ORB_SLAM2
{

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
  ORBVocabulary;

} //namespace ORB_SLAM

#endif // ORBVOCABULARY_H
