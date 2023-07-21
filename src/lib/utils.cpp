//
// Created by luc on 22/10/22.
//

#include "utils.h"

namespace vkb {


}

namespace ImGui {
    bool CSliderFloatRanged3(const char *label, float *v, float *v_min, float *v_max, const char *format, ImGuiSliderFlags flags) {
        ImGuiWindow* window = GetCurrentWindow();
        if (window->SkipItems)
            return false;

        ImGuiContext& g = *GImGui;
        bool value_changed = false;
        BeginGroup();
        PushID(label);
        PushMultiItemsWidths(3, CalcItemWidth());
        for (int i = 0; i < 3; i++)
        {
            PushID(i);
            if (i > 0)
                SameLine(0, g.Style.ItemInnerSpacing.x);
            value_changed |= SliderFloat("", v, *v_min, *v_max, format, flags);
            PopID();
            PopItemWidth();
            v++;
            v_min++;
            v_max++;
        }
        PopID();

        const char* label_end = FindRenderedTextEnd(label);
        if (label != label_end)
        {
            SameLine(0, g.Style.ItemInnerSpacing.x);
            TextEx(label, label_end);
        }

        EndGroup();
        return value_changed;
    }

    bool CDragFloatRanged3(const char *label, float *v, float v_speed, float *v_min, float *v_max, const char *format, ImGuiSliderFlags flags) {
        ImGuiWindow* window = GetCurrentWindow();
        if (window->SkipItems)
            return false;

        ImGuiContext& g = *GImGui;
        bool value_changed = false;
        BeginGroup();
        PushID(label);
        PushMultiItemsWidths(3, CalcItemWidth());
        for (int i = 0; i < 3; i++)
        {
            PushID(i);
            if (i > 0)
                SameLine(0, g.Style.ItemInnerSpacing.x);
            value_changed |= DragFloat("", v, v_speed, *v_min, *v_max, format, flags);
            PopID();
            PopItemWidth();
            v++;
            v_min++;
            v_max++;
        }
        PopID();

        const char* label_end = FindRenderedTextEnd(label);
        if (label != label_end)
        {
            SameLine(0, g.Style.ItemInnerSpacing.x);
            TextEx(label, label_end);
        }

        EndGroup();
        return value_changed;
    }

    bool CSliderIntRanged2(const char *label, int v[2], int v_min[2], int v_max[2], const char *format, ImGuiSliderFlags flags) {
        ImGuiWindow* window = GetCurrentWindow();
        if (window->SkipItems)
            return false;

        ImGuiContext& g = *GImGui;
        bool value_changed = false;
        BeginGroup();
        PushID(label);
        PushMultiItemsWidths(3, CalcItemWidth());
        for (int i = 0; i < 2; i++)
        {
            PushID(i);
            if (i > 0)
                SameLine(0, g.Style.ItemInnerSpacing.x);
            value_changed |= SliderInt("", v, *v_min, *v_max, format, flags);
            PopID();
            PopItemWidth();
            v++;
            v_min++;
            v_max++;
        }
        PopID();

        const char* label_end = FindRenderedTextEnd(label);
        if (label != label_end)
        {
            SameLine(0, g.Style.ItemInnerSpacing.x);
            TextEx(label, label_end);
        }

        EndGroup();
        return value_changed;
    }

}