#include "KeyProcess.h"
#include <vector>

/**
 * @ingroup TDT_ALG
 * @defgroup TDT_KEY_PROCESS DBUS按键处理
 * @brief 可以通过使用该类，方便的将单键和组合键与其对应的效果绑定起来，并且支持方便的修改键值
 * @details 为了解决单键和CTRL、SHIFT的组合问题，为此使用了两个非常拗口的变量进行设置—— keyProcessCtrlToDisable 和 keyProcessShiftToDisable 。  
 * 此外，该类还使用了一个纯C++特性——匿名函数(lambda函数)。因此，使用该类的难点在于上面两点。
 * @note 此类的主要用处在一些标志位的设定
 * @section init_use 定义和使用
 * @code {.cpp}
KeyProcess buffToggle(
	KEY_G, [](uint32_t *interval) {
		if (gimbal.desireMode == gimbal.buffMode)
		{
			quitBuff();
		}
		else
		{
			enterBuff();
		}
	},
	NULL, NULL, 1, 1); //单键，遇到ctrl和shift不处理
 * @endcode
 * 其中，
 * @code {.cpp}
 * [](uint32_t *interval) {
 * 	//...
 * }
 * @endcode
 * 是一个lambda函数，其实就是一个函数，它没有名字，通过函数指针来储存以及调用。我们只需要按照这个格式定义就行了。   
 * 其中有一个参数(uint32_t *interval)代表按住的持续时间（毫秒）（即高电平持续时间），如果是按下触发（上升沿捕获触发）那么当自动调用这个lambda函数时interval的值就是0。  
 * 我们知道打符触发是相当于一个上升沿捕获触发，即只需要按一下进入，再按一下退出，因此只需要填press(按下)的lambda函数 ( 其他填 NULL 或 nullptr )。
 * @warning 该键值扫描函数与dbus更新频率一致，即按住函数调用频率与dbus更新频率一致
 * 此时，不同按键对应效果如图所示  
<table>
<caption id="BuffToggleState">keyProcessCtrlToDisable=1,
keyProcessShiftToDisable=1,打符触发状态</caption>
<tr><th>键值  <th>打符触发状态
<tr><td>G <td>触发
<tr><td>G+A <td>触发
<tr><td>CTRL+G <td>不触发
<tr><td>CTRL+G+A <td>不触发
<tr><td>SHIFT+G <td>不触发
<tr><td>SHIFT+G+A <td>不触发
<tr><td>CTRL+SHIFT+G <td>不触发
</table>  
 * 换句话说，通过修改构造器的最后两项，能产生不同的效果
<table>
<tr><th>键值  <th>打符触发状态
<tr><td>G <td>
<table>
<tr><td>keyProcessCtrlToDisable=0<td>keyProcessShiftToDisable=0<td>触发
<tr><td>keyProcessCtrlToDisable=0<td>keyProcessShiftToDisable=1<td>触发
<tr><td>keyProcessCtrlToDisable=1<td>keyProcessShiftToDisable=0<td>触发
<tr><td>keyProcessCtrlToDisable=1<td>keyProcessShiftToDisable=1<td>触发
</table>
<tr><td>G+A <td>
<table>
<tr><td>keyProcessCtrlToDisable=0<td>keyProcessShiftToDisable=0<td>触发
<tr><td>keyProcessCtrlToDisable=0<td>keyProcessShiftToDisable=1<td>触发
<tr><td>keyProcessCtrlToDisable=1<td>keyProcessShiftToDisable=0<td>触发
<tr><td>keyProcessCtrlToDisable=1<td>keyProcessShiftToDisable=1<td>触发
</table>
<tr><td>CTRL+G <td>
<table>
<tr><td>keyProcessCtrlToDisable=0<td>keyProcessShiftToDisable=0<td>触发
<tr><td>keyProcessCtrlToDisable=0<td>keyProcessShiftToDisable=1<td>触发
<tr><td>keyProcessCtrlToDisable=1<td>keyProcessShiftToDisable=0<td>不触发
<tr><td>keyProcessCtrlToDisable=1<td>keyProcessShiftToDisable=1<td>不触发
</table>
<tr><td>CTRL+G+A <td>
<table>
<tr><td>keyProcessCtrlToDisable=0<td>keyProcessShiftToDisable=0<td>触发
<tr><td>keyProcessCtrlToDisable=0<td>keyProcessShiftToDisable=1<td>触发
<tr><td>keyProcessCtrlToDisable=1<td>keyProcessShiftToDisable=0<td>不触发
<tr><td>keyProcessCtrlToDisable=1<td>keyProcessShiftToDisable=1<td>不触发
</table>
<tr><td>SHIFT+G <td>
<table>
<tr><td>keyProcessCtrlToDisable=0<td>keyProcessShiftToDisable=0<td>触发
<tr><td>keyProcessCtrlToDisable=0<td>keyProcessShiftToDisable=1<td>不触发
<tr><td>keyProcessCtrlToDisable=1<td>keyProcessShiftToDisable=0<td>触发
<tr><td>keyProcessCtrlToDisable=1<td>keyProcessShiftToDisable=1<td>不触发
</table>
<tr><td>SHIFT+G+A <td>
<table>
<tr><td>keyProcessCtrlToDisable=0<td>keyProcessShiftToDisable=0<td>触发
<tr><td>keyProcessCtrlToDisable=0<td>keyProcessShiftToDisable=1<td>不触发
<tr><td>keyProcessCtrlToDisable=1<td>keyProcessShiftToDisable=0<td>触发
<tr><td>keyProcessCtrlToDisable=1<td>keyProcessShiftToDisable=1<td>不触发
</table>
<tr><td>CTRL+SHIFT+G <td>
<table>
<tr><td>keyProcessCtrlToDisable=0<td>keyProcessShiftToDisable=0<td>触发
<tr><td>keyProcessCtrlToDisable=0<td>keyProcessShiftToDisable=1<td>不触发
<tr><td>keyProcessCtrlToDisable=1<td>keyProcessShiftToDisable=0<td>不触发
<tr><td>keyProcessCtrlToDisable=1<td>keyProcessShiftToDisable=1<td>不触发
</table>
</table>  
 */

static std::vector<KeyProcess *> *keyList = 0;
static std::vector<uint16_t> *keyValueList = 0;

Cycle KeyProcess::keyCycle = Cycle();

KeyProcess::KeyProcess(uint16_t keyValue, void (*press)(uint32_t *interval), void (*release)(uint32_t *interval),
					   void (*hold)(uint32_t *interval), u8 keyProcessCtrlToDisable, u8 keyProcessShiftToDisable) : 
						   press(press), release(release), hold(hold), interval(0), _enable(1), hasPress(0), 
					   keyProcessCtrlToDisable(keyProcessCtrlToDisable), keyProcessShiftToDisable(keyProcessShiftToDisable)
{
	if (!keyList)
	{
		keyList = new std::vector<KeyProcess *>;
		keyValueList = new std::vector<uint16_t>;
		for (int i = 0; i < 16; i++)
		{
			keyValueList->push_back(1 << i);
			keyList->push_back(new KeyProcess());
		}
	}
	
	auto it = std::find(keyValueList->begin(), keyValueList->end(), keyValue);

	if (it != keyValueList->end())
	{
		int index = it - keyValueList->begin();
		delete (*keyList)[index];
		(*keyList)[index] = this;
		return;
	}

	if (keyValue & KEY_SHIFT) //如果有shift的按键
	{
		this->keyProcessShiftToDisable = 0;
	}

	if (keyValue & KEY_CTRL) //如果有ctrl的按键
	{
		this->keyProcessCtrlToDisable = 0;
	}

	keyValueList->push_back(keyValue);
	keyList->push_back(this);
}

KeyProcess::KeyProcess() : press(0), release(0), hold(0), interval(0), _enable(1), hasPress(0)
{
}

void KeyProcess::keyHandle(uint16_t keyValue)
{
	if (keyValueList == 0)
		return;
	uint16_t count = keyValueList->size();
	if (count == 0)
		return;

	uint32_t keyInterval = keyCycle.getCycleT() * 1000;
	if (keyInterval > 500)
		keyInterval = 0;

	keyTravel(keyValue, 0, count, keyInterval);
}

void KeyProcess::setEnable(u8 enable)
{
	if (enable)
		this->enable();
	else
		disable();
}

void KeyProcess::disable()
{
	this->_enable = 0;
	if (hasPress)
	{
		preRelease();
		hasPress = 0;
	}
}

void KeyProcess::keyTravel(uint16_t keyValue, uint16_t indexFrom, uint16_t indexTo, std::uint32_t interval)
{
	KeyProcess *item;
	uint16_t itemKeyValue;
	u8 ctrlPressed = ((keyValue & KEY_CTRL) == KEY_CTRL);
	u8 shiftPressed = ((keyValue & KEY_SHIFT) == KEY_SHIFT);
	for (uint16_t i = indexFrom; i < indexTo; i++)
	{
		item = (*keyList)[i];
		itemKeyValue = (*keyValueList)[i];
		if (shiftPressed && item->keyProcessShiftToDisable)
		{
			item->disable();
			continue;
		}
		else
		{
			item->enable();
		}

		if (ctrlPressed && item->keyProcessCtrlToDisable)
		{
			item->disable();
			continue;
		}
		else
		{
			item->enable();
		}

		if (itemKeyValue == (keyValue & itemKeyValue)) //按下
		{
			if (item->hasPress) //之前已经按下
			{
				item->interval += interval;
				item->preHold();
				continue;
			}
			item->hasPress = 1; //之前没按下
			item->prePress();
			continue;
		}
		if (item->hasPress)
		{
			item->hasPress = 0;
			item->preRelease();
		}
	}
}

KeyProcess *KeyProcess::getKeyProcess(uint16_t keyValue)
{
	int count = keyValueList->size();
	for (int i = 0; i < count; i++)
	{
		if (keyValue == (*keyValueList)[i])
			return (*keyList)[i];
	}
	return 0;
}
