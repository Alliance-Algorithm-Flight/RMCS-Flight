/**
 * @file src/util/armor_filter.cpp
 */

#include "util/armor_filter.hpp"

#include <algorithm>
#include <bit>

namespace rmcs_tongji_auto_aim::util {

namespace {

using world_exe::data::ArmorImageSpacing;
using world_exe::enumeration::ArmorIdFlag;
using world_exe::enumeration::CarIDFlag;
using world_exe::interfaces::IArmorInImage;

constexpr std::array<ArmorIdFlag, 8> kArmorOrder = {
    ArmorIdFlag::Hero,
    ArmorIdFlag::Engineer,
    ArmorIdFlag::InfantryIII,
    ArmorIdFlag::InfantryIV,
    ArmorIdFlag::InfantryV,
    ArmorIdFlag::Sentry,
    ArmorIdFlag::Base,
    ArmorIdFlag::Outpost,
};

constexpr uint8_t kWhitelistHero        = 0x1;
constexpr uint8_t kWhitelistEngineer    = 0x2;
constexpr uint8_t kWhitelistInfantryIII = 0x4;
constexpr uint8_t kWhitelistInfantryIV  = 0x8;
constexpr uint8_t kWhitelistInfantryV   = 0x10;
constexpr uint8_t kWhitelistSentry      = 0x20;
constexpr uint8_t kWhitelistOutpost     = 0x40;
constexpr uint8_t kWhitelistBase        = 0x80;

/**
 * @brief Determines whether a specific armor type is permitted by a whitelist mask.
 *
 * For Hero, Engineer, InfantryIII, InfantryIV, InfantryV, Sentry, and Outpost, the armor
 * is allowed when the corresponding whitelist bit is not set. For Base, the armor is
 * allowed only when the Base whitelist bit is set. All other armor flags are allowed.
 *
 * @param flag Armor type to check.
 * @param whitelist Bitmask where each bit corresponds to an armor type's whitelist setting.
 * @return `true` if the armor type is allowed by the mask, `false` otherwise.
 */
bool isArmorAllowed(ArmorIdFlag flag, uint8_t whitelist) {
    switch (flag) {
    case ArmorIdFlag::Hero: return (whitelist & kWhitelistHero) == 0;
    case ArmorIdFlag::Engineer: return (whitelist & kWhitelistEngineer) == 0;
    case ArmorIdFlag::InfantryIII: return (whitelist & kWhitelistInfantryIII) == 0;
    case ArmorIdFlag::InfantryIV: return (whitelist & kWhitelistInfantryIV) == 0;
    case ArmorIdFlag::InfantryV: return (whitelist & kWhitelistInfantryV) == 0;
    case ArmorIdFlag::Sentry: return (whitelist & kWhitelistSentry) == 0;
    case ArmorIdFlag::Outpost: return (whitelist & kWhitelistOutpost) == 0;
    case ArmorIdFlag::Base: return (whitelist & kWhitelistBase) != 0;
    default: return true;
    }
}

class FilteredArmorInImage : public IArmorInImage {
public:
    /**
     * @brief Constructs a filtered view of armors from a source image according to a whitelist.
     *
     * Creates an object that preserves the source timestamp and stores, for each armor
     * type in the fixed processing order, any non-empty armor lists from the source
     * that are allowed by the provided whitelist mask.
     *
     * @param source Shared pointer to the source IArmorInImage; must be non-null.
     * @param whitelist Bitmask controlling which armor types are allowed in the filtered view.
     */
    FilteredArmorInImage(
        const std::shared_ptr<IArmorInImage>& source, uint8_t whitelist)
        : timestamp_(source->GetTimeStamp()) {
        for (size_t idx = 0; idx < kArmorOrder.size(); ++idx) {
            const auto flag = kArmorOrder[idx];
            if (!isArmorAllowed(flag, whitelist)) continue;
            const auto& armors = source->GetArmors(flag);
            if (armors.empty()) continue;
            filtered_[idx] = armors;
        }
    }

    /**
 * @brief Retrieves the stored timestamp for the armor image.
 *
 * The timestamp is captured from the source IArmorInImage and preserved by this filtered view.
 *
 * @return const world_exe::data::TimeStamp& The stored timestamp of the source image.
 */
const world_exe::data::TimeStamp& GetTimeStamp() const override { return timestamp_; }

    /**
     * @brief Retrieves filtered armors for the specified armor type.
     *
     * Returns the stored list of ArmorImageSpacing for the given ArmorIdFlag if present; otherwise returns a reference to an empty vector.
     *
     * @param armor_id Armor type identifier to query.
     * @return const std::vector<ArmorImageSpacing>& The vector of filtered armors for the requested armor type, or an empty vector reference when no armors are stored for that type.
     */
    const std::vector<ArmorImageSpacing>& GetArmors(
        const ArmorIdFlag& armor_id) const override {
        for (size_t idx = 0; idx < kArmorOrder.size(); ++idx) {
            if (armor_id == kArmorOrder[idx]) {
                return filtered_[idx];
            }
        }
        return empty_;
    }

private:
    world_exe::data::TimeStamp timestamp_;
    std::array<std::vector<ArmorImageSpacing>, kArmorOrder.size()> filtered_{};
    std::vector<ArmorImageSpacing> empty_;
};

}  /**
 * @brief Constructs a filtered armor view and computes car ID flags for armor present after filtering.
 *
 * Filters armor entries from the provided source according to the bitmask whitelist and returns
 * an ArmorFilterResult containing a filtered armor accessor and a CarIDFlag bitfield that marks
 * which armor types are present in the filtered data.
 *
 * @param source Shared pointer to the source armor image; if null, an empty/default ArmorFilterResult is returned.
 * @param whitelist Bitmask controlling which armor types are included (bits correspond to the kWhitelist* constants).
 * @return ArmorFilterResult `armors` holds a FilteredArmorInImage reflecting the whitelist (or is default/empty if source was null);
 *         `car_id_flag` is the bitwise OR of CarIDFlag values for armor types present after filtering.
 */

ArmorFilterResult filterArmorsByWhitelist(
    const std::shared_ptr<IArmorInImage>& source, uint8_t whitelist) {
    ArmorFilterResult result;
    if (!source) {
        return result;
    }

    auto filtered = std::make_shared<FilteredArmorInImage>(source, whitelist);
    CarIDFlag flags = CarIDFlag::None;

    for (size_t idx = 0; idx < kArmorOrder.size(); ++idx) {
        if (!filtered->GetArmors(kArmorOrder[idx]).empty()) {
            flags = static_cast<CarIDFlag>(
                static_cast<uint32_t>(flags) | static_cast<uint32_t>(kArmorOrder[idx]));
        }
    }

    result.armors = std::move(filtered);
    result.car_id_flag = flags;
    return result;
}

}  // namespace rmcs_tongji_auto_aim::util