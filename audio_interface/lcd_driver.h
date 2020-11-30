#include <array>
#include <U8g2lib.h>
#include <Bounce2.h>

typedef U8G2_SH1107_SEEED_128X128_F_HW_I2C U8G2Type;

enum class LCDMenuType
{
  NONE = 0,
  MAIN,
  SD_FILE,
  SAMPLE_MENU,
};

class LCDMenu
{
private:
  static constexpr int Y_SPACING = 10;
  static constexpr int X_SPACING = 10;

  static constexpr size_t MAX_ENTRY_LEN = 20;
  static constexpr size_t MAX_ENTRIES = 10;
  char name_[MAX_ENTRY_LEN];

  LCDMenuType menu_type_ = LCDMenuType::NONE;

  LCDMenu* parent_ = nullptr;
  std::array<LCDMenu*, MAX_ENTRIES> children_ = {}; // Defaults to nullptr

  String title_;
  std::array<String, MAX_ENTRIES> entries_;

  int valid_entries_ = 0;

protected:

  void setEntries(const std::array<String, MAX_ENTRIES>& entries)
  {
    entries_  = entries;
    // Initialize valid_entries for choice highlighting purposes
    valid_entries_ = 0;
    while (entries[valid_entries_].length() > 0)
      ++valid_entries_;
  }

  void setTitle(const String& title)
  {
    title_ = title;
  }

  void setType(const LCDMenuType& menu_type)
  {
    menu_type_ = menu_type;
  }

public:
  void updateEntry(int& entry)
  {
    entry = entry < 0 ? 0 : (entry < (valid_entries_ - 1) ? entry : valid_entries_ - 1);
  }

  void print(U8G2Type& u8g2, int& entry)
  {
    // Boundary checking
    u8g2.clearBuffer();
    int line_idx = 1;
    u8g2.drawStr(X_SPACING, line_idx * Y_SPACING, title_.c_str());
    // Skip one line
    line_idx += 2;
    for (int i = 0; i < valid_entries_; ++i, ++line_idx)
    {
      if (i == entry)
      {
        int circle_radius = X_SPACING / 2 - 1;
        u8g2.drawCircle(X_SPACING / 2, line_idx * (Y_SPACING) - circle_radius, circle_radius);
      }
      u8g2.drawStr(X_SPACING, line_idx * Y_SPACING, entries_[i].c_str());	// write something to the internal memory
    }
    u8g2.sendBuffer();					// transfer internal memory to the display
    
  }

  LCDMenuType getType() const
  {
    return menu_type_;
  }

  LCDMenu* getChild(int entry) const
  {
    return children_[entry];
  }

  LCDMenu* getParent() const
  {
    return parent_;
  }

  void setParent(LCDMenu* parent)
  {
    parent_ = parent;
  }

  void attachChild(LCDMenu* child, int idx)
  {
    child->setParent(this);
    children_[idx] = child;
  }
};

class LCDMainMenu : public LCDMenu
{
public:
  LCDMainMenu()
  {
    setTitle("Main Menu");
    setType(LCDMenuType::MAIN);
    setEntries({"Loop base", "Second functionality", "Third functionality"});
  }
};

class LCDSdMenu : public LCDMenu
{
public:
  LCDSdMenu()
  {
    setTitle("SD Files");
    setType(LCDMenuType::SD_FILE);
    setEntries({"First subfunction", "Second subfunction", "Third subfunction"});
  }
};

class AudioLCD
{
public:
  AudioLCD() : 
    u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE),
    current_menu_(&main_menu_)
  {
    main_menu_.attachChild(&sd_menu_, 0);
    u8g2.begin();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    sw_next.attach(SW_NEXT, INPUT_PULLUP);
    sw_prev.attach(SW_PREV, INPUT_PULLUP);
    sw_select.attach(SW_SELECT, INPUT_PULLUP);
    sw_cancel.attach(SW_CANCEL, INPUT_PULLUP);
    sw_next.interval(BOUNCE_INTERVAL);
    sw_prev.interval(BOUNCE_INTERVAL);
    sw_select.interval(BOUNCE_INTERVAL);
    sw_cancel.interval(BOUNCE_INTERVAL);
  }

  void printMenu(int& entry)
  {
    current_menu_->updateEntry(entry);
    current_menu_->print(u8g2, entry);
  }

  void update()
  {
    auto event = getButtonEvent();
    auto menu_type = current_menu_->getType();
    if (state == LCDState::IN_MENU)
    {
      // Scroll the menu
      // TODO checking on number of entries, multiple pages
      if (event == ButtonEvent::NEXT)
        printMenu(++selected_entry_);
      else if (event == ButtonEvent::PREV)
        printMenu(--selected_entry_);
      else if (event == ButtonEvent::SELECT)
      {
        // State Transition
        auto menu_child = current_menu_->getChild(selected_entry_);
        if (menu_child != nullptr)
        {
          current_menu_ = menu_child;
          selected_entry_ = 0;
          printMenu(selected_entry_);
        }
      }
      else if (event == ButtonEvent::CANCEL)
      {
        auto menu_parent = current_menu_->getParent();
        if (menu_parent != nullptr)
        {
          current_menu_ = menu_parent;
          selected_entry_ = 0;
          printMenu(selected_entry_);
        }
      }
    }
    else
    {
      // Noop, just start / stop?
    }
  }


private:
  enum class LCDState
  {
    IN_MENU,
    RUNNING,
  } state;

  U8G2_SH1107_SEEED_128X128_F_HW_I2C u8g2;

  Bounce sw_next, sw_prev, sw_select, sw_cancel;

  int selected_entry_ = 0;

  LCDMainMenu main_menu_;
  LCDSdMenu sd_menu_;
  LCDMenu* current_menu_;

  static constexpr int BOUNCE_INTERVAL = 10;

  void updateButtons()
  {
    sw_next.update();
    sw_prev.update();
    sw_select.update();
    sw_cancel.update();
  }

  enum class ButtonEvent
  {
    NONE,
    NEXT,
    PREV,
    SELECT,
    CANCEL,
  };

  std::array<String, FILES_PER_PAGE> sd_files;

  ButtonEvent getButtonEvent()
  {
    updateButtons();
    ButtonEvent ev = ButtonEvent::NONE;
    if (sw_next.fell())
      ev = ButtonEvent::NEXT;
    else if (sw_prev.fell())
      ev = ButtonEvent::PREV;
    else if (sw_select.fell())
      ev = ButtonEvent::SELECT;
    else if (sw_cancel.fell())
      ev = ButtonEvent::CANCEL;
    return ev;
  }
};
