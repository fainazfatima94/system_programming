// gtest
#include <gtest/gtest.h>   // googletest header file

// gmlib2
#include <core/projectivespaceobject.h>
using namespace gmlib2::spaces;


TEST(Core_ProjectiveSpaceObjects_Traits,
     ProjectiveSpaceObject_isProjectiveSpaceObject)
{

  using ProjectiveSpaceObject = gmlib2::ProjectiveSpaceObject<>;

  EXPECT_TRUE(
    gmlib2::traits::Is_ProjectiveSpaceObject<ProjectiveSpaceObject>::value);

  EXPECT_TRUE(
    gmlib2::traits::Is_SpaceInfo<ProjectiveSpaceObject::EmbedSpaceInfo>::value);

  EXPECT_TRUE(gmlib2::traits::Is_ProjectiveSpace<
              ProjectiveSpaceObject::EmbedSpace>::value);
}
