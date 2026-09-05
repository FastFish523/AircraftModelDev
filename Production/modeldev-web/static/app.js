(() => {
  "use strict";

  const API_BASE = "/api";
  const POLL_INTERVAL_MS = 1000;
  const REQUEST_TIMEOUT_MS = 15000;
  const MAX_POLL_ERRORS = 5;
  const SVG_NS = "http://www.w3.org/2000/svg";

  const elements = {
    workspaceTitle: document.querySelector("#workspace-title"),
    workspaceSubtitle: document.querySelector("#workspace-subtitle"),
    workspaceCrumb: document.querySelector("#workspace-crumb"),
    navItems: [...document.querySelectorAll("[data-screen-target]")],
    screens: [...document.querySelectorAll("[data-workspace-screen]")],
    connectionDot: document.querySelector("#connection-dot"),
    connectionLabel: document.querySelector("#connection-label"),
    topRunState: document.querySelector("#top-run-state"),
    globalError: document.querySelector("#global-error"),
    globalErrorMessage: document.querySelector("#global-error-message"),
    dismissError: document.querySelector("#dismiss-error"),
    reloadCatalog: document.querySelector("#reload-catalog"),
    catalogSummary: document.querySelector("#catalog-summary"),
    catalogEmpty: document.querySelector("#catalog-empty"),
    catalogContent: document.querySelector("#catalog-content"),
    catalogCategoryCount: document.querySelector("#catalog-category-count"),
    catalogCategoryList: document.querySelector("#catalog-category-list"),
    catalogCategoryDescription: document.querySelector("#catalog-category-description"),
    catalogModelCount: document.querySelector("#catalog-model-count"),
    catalogModelList: document.querySelector("#catalog-model-list"),
    catalogDetailStatus: document.querySelector("#catalog-detail-status"),
    catalogDetailBody: document.querySelector("#catalog-detail-body"),
    reloadModels: document.querySelector("#reload-models"),
    runForm: document.querySelector("#run-form"),
    modelSelect: document.querySelector("#model-select"),
    modelHelp: document.querySelector("#model-help"),
    scenarioName: document.querySelector("#scenario-name"),
    parameterFields: document.querySelector("#parameter-fields"),
    parameterEmpty: document.querySelector("#parameter-empty"),
    moduleFields: document.querySelector("#module-fields"),
    moduleEmpty: document.querySelector("#module-empty"),
    resetParameters: document.querySelector("#reset-parameters"),
    runButton: document.querySelector("#run-button"),
    formMessage: document.querySelector("#form-message"),
    runStatus: document.querySelector("#run-status"),
    cancelRun: document.querySelector("#cancel-run"),
    runId: document.querySelector("#run-id"),
    runState: document.querySelector("#run-state"),
    simTime: document.querySelector("#sim-time"),
    updatedAt: document.querySelector("#updated-at"),
    progressTrack: document.querySelector("#progress-track"),
    progressBar: document.querySelector("#progress-bar"),
    progressLabel: document.querySelector("#progress-label"),
    logOutput: document.querySelector("#log-output"),
    clearLogs: document.querySelector("#clear-logs"),
    resultsCaption: document.querySelector("#results-caption"),
    resultsEmpty: document.querySelector("#results-empty"),
    resultsContent: document.querySelector("#results-content"),
    resultActions: document.querySelector("#result-actions"),
    copyResult: document.querySelector("#copy-result"),
    downloadResultJson: document.querySelector("#download-result-json"),
    downloadResultCsv: document.querySelector("#download-result-csv"),
    downloadResultRaw: document.querySelector("#download-result-raw"),
    altitudeChart: document.querySelector("#altitude-chart"),
    speedChart: document.querySelector("#speed-chart"),
    trajectoryChart: document.querySelector("#trajectory-chart"),
    trajectoryCaption: document.querySelector("#trajectory-caption"),
    summaryGrid: document.querySelector("#summary-grid"),
    reloadHistory: document.querySelector("#reload-history"),
    historyEmpty: document.querySelector("#history-empty"),
    historyContent: document.querySelector("#history-content"),
    runHistoryBody: document.querySelector("#run-history-body"),
    compareSelectionCount: document.querySelector("#compare-selection-count"),
    compareRuns: document.querySelector("#compare-runs"),
    comparisonCaption: document.querySelector("#comparison-caption"),
    comparisonEmpty: document.querySelector("#comparison-empty"),
    comparisonContent: document.querySelector("#comparison-content"),
    comparisonLegend: document.querySelector("#comparison-legend"),
    comparisonAltitudeChart: document.querySelector("#comparison-altitude-chart"),
    comparisonSpeedChart: document.querySelector("#comparison-speed-chart"),
    comparisonMetricsHead: document.querySelector("#comparison-metrics-head"),
    comparisonMetricsBody: document.querySelector("#comparison-metrics-body"),
  };

  const state = {
    runId: null,
    runToken: 0,
    pollTimer: null,
    pollErrors: 0,
    running: false,
    latestResult: null,
    logs: [],
    logSet: new Set(),
    models: [],
    generatedScenarioName: "",
    history: [],
    selectedCompareIds: new Set(),
    resultCache: new Map(),
    comparisonRunning: false,
    catalogCategories: [],
    activeCatalogCategoryId: null,
    activeCatalogModelId: null,
    activeScreen: "catalog",
  };

  const SUCCESS_STATES = new Set(["completed", "complete", "succeeded", "success", "finished", "done"]);
  const FAILURE_STATES = new Set(["failed", "failure", "error", "cancelled", "canceled", "timeout", "timed_out"]);
  const QUEUED_STATES = new Set(["queued", "pending", "created", "waiting", "accepted"]);
  const COMPARISON_COLORS = ["#1677ff", "#0bb7c8", "#6b5fe3", "#e58a00"];
  const SCREEN_META = {
    catalog: ["模型库", "浏览已发现的模型类别、集成状态与运行能力"],
    scenario: ["场景配置", "选择运行模型并配置场景、制导与控制参数"],
    runtime: ["运行监控", "查看任务状态、仿真进度与服务端日志"],
    results: ["结果分析", "检查运行曲线、轨迹、摘要与原始数据"],
    history: ["运行历史", "载入、导出并选择已有运行记录"],
    compare: ["结果对比", "叠加比较同模型的多次成功运行"],
  };

  function showScreen(screenId, { focus = false } = {}) {
    if (!SCREEN_META[screenId]) return;
    state.activeScreen = screenId;
    for (const screen of elements.screens) screen.hidden = screen.dataset.workspaceScreen !== screenId;
    for (const item of elements.navItems) {
      const selected = item.dataset.screenTarget === screenId;
      item.classList.toggle("is-active", selected);
      item.setAttribute("aria-selected", selected ? "true" : "false");
    }
    const [title, subtitle] = SCREEN_META[screenId];
    elements.workspaceTitle.textContent = title;
    elements.workspaceSubtitle.textContent = subtitle;
    elements.workspaceCrumb.textContent = title;
    if (screenId === "history") void loadHistory({ silent: true });
    if (focus) document.querySelector(`[data-workspace-screen="${screenId}"] h2`)?.focus?.();
    window.scrollTo({ top: 0, behavior: "smooth" });
  }

  function catalogCategoryItems(payload) {
    const source = Array.isArray(payload)
      ? payload
      : payload && (payload.categories || payload.data?.categories || payload.data);
    if (!Array.isArray(source)) return [];
    return source.map((category, categoryIndex) => {
      if (!category || typeof category !== "object") return null;
      const id = String(category.id ?? category.key ?? category.name ?? `category-${categoryIndex + 1}`).trim();
      const rawModels = Array.isArray(category.models) ? category.models : [];
      const models = rawModels.map((model, modelIndex) => {
        if (!model || typeof model !== "object") return null;
        const modelId = String(model.id ?? model.model_id ?? model.name ?? `model-${modelIndex + 1}`).trim();
        const rawStatus = String(model.status ?? (model.run_model_id ? "runnable" : "not_integrated")).trim().toLowerCase();
        const status = ["runnable", "build_required", "not_integrated"].includes(rawStatus)
          ? rawStatus
          : "not_integrated";
        return {
          ...model,
          id: modelId,
          name: String(model.name ?? model.display_name ?? model.label ?? modelId),
          description: String(model.description ?? ""),
          status,
          statusLabel: String(model.status_label ?? ({
            runnable: "可运行",
            build_required: "需要构建",
            not_integrated: "尚未集成",
          })[status]),
          runModelId: model.run_model_id == null ? "" : String(model.run_model_id),
          sourcePath: model.source_path == null ? "" : String(model.source_path),
          version: model.version == null ? "" : String(model.version),
        };
      }).filter((model) => model?.id);
      return {
        ...category,
        id,
        name: String(category.name ?? category.label ?? id),
        description: String(category.description ?? ""),
        models,
      };
    }).filter((category) => category?.id);
  }

  function activeCatalogCategory() {
    return state.catalogCategories.find((category) => category.id === state.activeCatalogCategoryId)
      || state.catalogCategories[0]
      || null;
  }

  function activeCatalogModel(category = activeCatalogCategory()) {
    return category?.models.find((model) => model.id === state.activeCatalogModelId)
      || category?.models[0]
      || null;
  }

  function catalogStatusClass(status) {
    if (status === "runnable") return "is-runnable";
    if (status === "build_required") return "is-build-required";
    return "is-not-integrated";
  }

  function appendCatalogValue(container, titleText, value) {
    if (value == null || value === "" || (Array.isArray(value) && !value.length)) return;
    const block = document.createElement("section");
    block.className = "catalog-detail-section";
    const title = document.createElement("h4");
    title.textContent = titleText;
    block.append(title);
    if (Array.isArray(value)) {
      const chips = document.createElement("div");
      chips.className = "catalog-chip-list";
      for (const item of value) {
        const chip = document.createElement("span");
        chip.textContent = typeof item === "object"
          ? String(item.name ?? item.label ?? item.id ?? JSON.stringify(item))
          : String(item);
        chips.append(chip);
      }
      block.append(chips);
    } else if (typeof value === "object") {
      const list = document.createElement("dl");
      list.className = "catalog-contract-list";
      for (const [key, item] of Object.entries(value)) {
        const term = document.createElement("dt");
        const description = document.createElement("dd");
        term.textContent = key;
        description.textContent = Array.isArray(item) ? item.join("、") : String(item ?? "—");
        list.append(term, description);
      }
      block.append(list);
    } else {
      const text = document.createElement("p");
      text.textContent = String(value);
      block.append(text);
    }
    container.append(block);
  }

  async function useCatalogModel(model) {
    if (model.status !== "runnable" || !model.runModelId) return;
    hideError();
    if (!state.models.some((item) => item.id === model.runModelId)) await loadModels();
    const runModel = state.models.find((item) => item.id === model.runModelId && item.available);
    if (!runModel) {
      showError(`目录中的“${model.name}”标记为可运行，但 GET /api/models 未返回运行模型 ${model.runModelId}。`);
      return;
    }
    elements.modelSelect.value = runModel.id;
    updateModelHelp();
    elements.formMessage.textContent = `已从模型库选择“${model.name}”，可以配置并启动仿真。`;
    elements.formMessage.classList.remove("is-error");
    showScreen("scenario");
    elements.scenarioName.focus();
  }

  function renderCatalogDetail(model) {
    elements.catalogDetailBody.replaceChildren();
    if (!model) {
      elements.catalogDetailStatus.textContent = "—";
      elements.catalogDetailStatus.className = "catalog-status";
      const empty = document.createElement("div");
      empty.className = "catalog-detail-empty";
      const title = document.createElement("strong");
      title.id = "catalog-detail-title";
      title.textContent = "当前类别没有模型";
      const text = document.createElement("p");
      text.textContent = "服务端没有为该类别返回模型条目。";
      empty.append(title, text);
      elements.catalogDetailBody.append(empty);
      return;
    }
    elements.catalogDetailStatus.textContent = model.statusLabel;
    elements.catalogDetailStatus.className = `catalog-status ${catalogStatusClass(model.status)}`;

    const heading = document.createElement("div");
    heading.className = "catalog-detail-heading";
    const text = document.createElement("div");
    const title = document.createElement("h3");
    title.id = "catalog-detail-title";
    title.textContent = model.name;
    const id = document.createElement("code");
    id.textContent = model.id;
    text.append(title, id);
    if (model.status === "runnable" && model.runModelId) {
      const useButton = document.createElement("button");
      useButton.type = "button";
      useButton.className = "button button-primary";
      useButton.textContent = "用于仿真";
      useButton.addEventListener("click", () => void useCatalogModel(model));
      heading.append(text, useButton);
    } else {
      heading.append(text);
    }
    elements.catalogDetailBody.append(heading);
    if (model.description) {
      const description = document.createElement("p");
      description.className = "catalog-description";
      description.textContent = model.description;
      elements.catalogDetailBody.append(description);
    }

    const facts = document.createElement("dl");
    facts.className = "catalog-facts";
    const factItems = [
      ["集成状态", model.statusLabel],
      ["运行模型 ID", model.runModelId || "—"],
      ["版本", model.version || "—"],
      ["源码检测", model.source_present === true ? "已发现" : model.source_present === false ? "未发现" : "未提供"],
      ["源码路径", model.sourcePath || "—"],
    ];
    for (const [label, value] of factItems) {
      const wrapper = document.createElement("div");
      const term = document.createElement("dt");
      const description = document.createElement("dd");
      term.textContent = label;
      description.textContent = value;
      wrapper.append(term, description);
      facts.append(wrapper);
    }
    elements.catalogDetailBody.append(facts);
    appendCatalogValue(elements.catalogDetailBody, "能力", model.capabilities);
    appendCatalogValue(elements.catalogDetailBody, "接口契约", model.contracts);
    appendCatalogValue(elements.catalogDetailBody, "集成证据", model.evidence);
  }

  function renderCatalogModels() {
    const category = activeCatalogCategory();
    if (category) state.activeCatalogCategoryId = category.id;
    const model = activeCatalogModel(category);
    state.activeCatalogModelId = model?.id || null;
    elements.catalogCategoryDescription.textContent = category?.description || "";
    elements.catalogModelCount.textContent = String(category?.models.length || 0);
    elements.catalogModelList.replaceChildren();
    for (const item of category?.models || []) {
      const button = document.createElement("button");
      button.type = "button";
      button.className = "catalog-model-item";
      button.dataset.modelId = item.id;
      button.setAttribute("aria-pressed", item.id === state.activeCatalogModelId ? "true" : "false");
      const main = document.createElement("span");
      const name = document.createElement("strong");
      const description = document.createElement("small");
      name.textContent = item.name;
      description.textContent = item.description || `模型 ID：${item.id}`;
      main.append(name, description);
      const status = document.createElement("span");
      status.className = `catalog-status ${catalogStatusClass(item.status)}`;
      status.textContent = item.statusLabel;
      button.append(main, status);
      button.addEventListener("click", () => {
        state.activeCatalogModelId = item.id;
        renderCatalogModels();
      });
      elements.catalogModelList.append(button);
    }
    if (!category?.models.length) {
      const empty = document.createElement("p");
      empty.className = "catalog-list-empty";
      empty.textContent = "该类别当前没有模型条目。";
      elements.catalogModelList.append(empty);
    }
    renderCatalogDetail(model);
  }

  function renderCatalog() {
    const categories = state.catalogCategories;
    const totalModels = categories.reduce((sum, category) => sum + category.models.length, 0);
    const runnableModels = categories.reduce((sum, category) => sum + category.models.filter((model) => model.status === "runnable").length, 0);
    elements.catalogSummary.textContent = `${categories.length} 个类别 · ${totalModels} 个模型 · ${runnableModels} 个可运行`;
    elements.catalogCategoryCount.textContent = String(categories.length);
    elements.catalogCategoryList.replaceChildren();
    if (!categories.length) {
      elements.catalogContent.hidden = true;
      elements.catalogEmpty.hidden = false;
      elements.catalogEmpty.querySelector("strong").textContent = "模型库为空";
      elements.catalogEmpty.querySelector("p").textContent = "服务端未返回任何模型类别。";
      return;
    }
    if (!categories.some((category) => category.id === state.activeCatalogCategoryId)) {
      state.activeCatalogCategoryId = categories[0].id;
      state.activeCatalogModelId = categories[0].models[0]?.id || null;
    }
    for (const category of categories) {
      const button = document.createElement("button");
      button.type = "button";
      button.className = "catalog-category-item";
      button.setAttribute("aria-pressed", category.id === state.activeCatalogCategoryId ? "true" : "false");
      const text = document.createElement("span");
      const name = document.createElement("strong");
      const description = document.createElement("small");
      name.textContent = category.name;
      description.textContent = category.description || `类别 ID：${category.id}`;
      text.append(name, description);
      const count = document.createElement("span");
      count.className = "catalog-count";
      count.textContent = String(category.models.length);
      button.append(text, count);
      button.addEventListener("click", () => {
        state.activeCatalogCategoryId = category.id;
        state.activeCatalogModelId = category.models[0]?.id || null;
        renderCatalog();
      });
      elements.catalogCategoryList.append(button);
    }
    elements.catalogEmpty.hidden = true;
    elements.catalogContent.hidden = false;
    renderCatalogModels();
  }

  async function loadModelCatalog() {
    elements.reloadCatalog.disabled = true;
    elements.catalogSummary.textContent = "正在读取目录…";
    try {
      const { payload } = await apiRequest("/model-catalog");
      state.catalogCategories = catalogCategoryItems(payload);
      renderCatalog();
    } catch (error) {
      elements.catalogContent.hidden = true;
      elements.catalogEmpty.hidden = false;
      elements.catalogEmpty.querySelector("strong").textContent = "模型库加载失败";
      elements.catalogEmpty.querySelector("p").textContent = errorMessage(error);
      elements.catalogSummary.textContent = "目录不可用";
    } finally {
      elements.reloadCatalog.disabled = false;
    }
  }

  function setConnection(online, label) {
    elements.connectionDot.classList.toggle("is-online", online === true);
    elements.connectionDot.classList.toggle("is-offline", online === false);
    elements.connectionLabel.textContent = label;
  }

  function showError(message) {
    elements.globalErrorMessage.textContent = message || "发生未知错误。";
    elements.globalError.hidden = false;
  }

  function hideError() {
    elements.globalError.hidden = true;
    elements.globalErrorMessage.textContent = "";
  }

  function errorMessage(error) {
    if (error instanceof Error && error.message) return error.message;
    if (typeof error === "string") return error;
    try {
      return JSON.stringify(error);
    } catch {
      return "发生未知错误。";
    }
  }

  async function apiRequest(path, options = {}) {
    const controller = new AbortController();
    const timer = window.setTimeout(() => controller.abort(), REQUEST_TIMEOUT_MS);
    const headers = new Headers(options.headers || {});
    if (options.body != null && !headers.has("Content-Type")) {
      headers.set("Content-Type", "application/json");
    }
    headers.set("Accept", "application/json");

    try {
      const response = await fetch(`${API_BASE}${path}`, {
        ...options,
        headers,
        signal: controller.signal,
      });
      const text = await response.text();
      let payload = null;
      if (text.trim()) {
        try {
          payload = JSON.parse(text);
        } catch {
          payload = text;
        }
      }
      if (!response.ok) {
        const detail = payload && typeof payload === "object"
          ? payload.detail || payload.error || payload.message
          : payload;
        throw new Error(detail || `请求失败（HTTP ${response.status}）`);
      }
      setConnection(true, "服务可用");
      return { payload, response };
    } catch (error) {
      if (error && error.name === "AbortError") {
        throw new Error("请求超时，请检查 ModelDev 服务是否正在运行。");
      }
      throw error;
    } finally {
      window.clearTimeout(timer);
    }
  }

  function modelItems(payload) {
    const source = Array.isArray(payload)
      ? payload
      : payload && (payload.models || payload.items || payload.data);
    if (!Array.isArray(source)) return [];
    return source.map((item) => {
      if (typeof item === "string") return { id: item, label: item, description: "", available: true };
      const id = item.id ?? item.model_id ?? item.key ?? item.name;
      const name = item.display_name ?? item.label ?? item.name ?? id;
      const version = item.version ? ` · ${item.version}` : "";
      return {
        id: String(id ?? ""),
        name: String(name ?? id ?? "未命名模型"),
        label: `${name ?? "未命名模型"}${version}`,
        description: String(item.description ?? ""),
        available: item.available !== false,
        parameters: parameterItems(item.parameters),
        modules: moduleItems(item.modules),
      };
    }).filter((item) => item.id);
  }

  function parameterItems(value) {
    if (!Array.isArray(value)) return [];
    const seen = new Set();
    return value.map((item) => {
      if (!item || typeof item !== "object") return null;
      const id = String(item.id ?? item.key ?? item.name ?? "").trim();
      if (!id || seen.has(id)) return null;
      seen.add(id);
      const defaultValue = Number(item.default);
      const rawMinimum = item.minimum ?? item.min;
      const rawMaximum = item.maximum ?? item.max;
      return {
        id,
        name: String(item.name ?? item.label ?? id),
        description: String(item.description ?? ""),
        unit: String(item.unit ?? ""),
        defaultValue: Number.isFinite(defaultValue) ? defaultValue : 0,
        minimum: rawMinimum != null && Number.isFinite(Number(rawMinimum)) ? Number(rawMinimum) : null,
        maximum: rawMaximum != null && Number.isFinite(Number(rawMaximum)) ? Number(rawMaximum) : null,
        step: Number.isFinite(Number(item.step)) && Number(item.step) > 0 ? Number(item.step) : "any",
      };
    }).filter(Boolean);
  }

  function moduleOptionItems(value) {
    if (!Array.isArray(value)) return [];
    const seen = new Set();
    return value.map((item) => {
      if (!item || typeof item !== "object") return null;
      const id = String(item.id ?? item.key ?? item.name ?? "").trim();
      if (!id || seen.has(id)) return null;
      seen.add(id);
      return {
        id,
        name: String(item.name ?? item.label ?? id),
        description: String(item.description ?? ""),
        parameters: parameterItems(item.parameters),
      };
    }).filter(Boolean);
  }

  function moduleItems(value) {
    if (!Array.isArray(value)) return [];
    const seen = new Set();
    return value.map((item) => {
      if (!item || typeof item !== "object") return null;
      const id = String(item.id ?? item.key ?? item.name ?? "").trim();
      if (!id || seen.has(id)) return null;
      seen.add(id);
      const options = moduleOptionItems(item.options);
      if (!options.length) return null;
      const requestedDefault = String(item.default ?? item.default_id ?? item.default_option_id ?? "");
      const defaultOptionId = options.some((option) => option.id === requestedDefault)
        ? requestedDefault
        : options[0].id;
      return {
        id,
        name: String(item.name ?? item.label ?? id),
        description: String(item.description ?? ""),
        defaultOptionId,
        options,
      };
    }).filter(Boolean);
  }

  function runModuleItems(value) {
    if (!value || typeof value !== "object") return [];
    const entries = Array.isArray(value)
      ? value.map((item, index) => [item?.slot_id ?? item?.slot ?? item?.module_id ?? `module_${index + 1}`, item])
      : Object.entries(value);
    return entries.map(([slotKey, item]) => {
      const slotId = String(slotKey ?? "").trim();
      if (!slotId) return null;
      if (typeof item === "string") {
        return { slot_id: slotId, id: item, name: "", parameters: {} };
      }
      if (!item || typeof item !== "object") return null;
      const optionId = String(item.id ?? item.option_id ?? item.option ?? "").trim();
      if (!optionId) return null;
      return {
        slot_id: slotId,
        slot_name: String(item.slot_name ?? ""),
        id: optionId,
        name: String(item.name ?? item.option_name ?? ""),
        parameters: item.parameters && typeof item.parameters === "object" && !Array.isArray(item.parameters)
          ? item.parameters
          : {},
      };
    }).filter(Boolean);
  }

  function selectedModel() {
    return state.models.find((item) => item.id === elements.modelSelect.value) || null;
  }

  function modelHasConfig(model) {
    return Boolean(model && ((model.parameters?.length || 0) + (model.modules?.length || 0)));
  }

  function createNumericParameterField(parameter, dataset) {
    const label = document.createElement("label");
    label.className = "field parameter-field";

    const title = document.createElement("span");
    title.textContent = parameter.name;
    if (parameter.unit) {
      const unit = document.createElement("em");
      unit.textContent = ` ${parameter.unit}`;
      title.append(unit);
    }

    const input = document.createElement("input");
    input.type = "number";
    input.required = true;
    for (const [key, value] of Object.entries(dataset)) input.dataset[key] = value;
    input.value = String(parameter.defaultValue);
    input.defaultValue = String(parameter.defaultValue);
    input.step = String(parameter.step);
    if (parameter.minimum != null) input.min = String(parameter.minimum);
    if (parameter.maximum != null) input.max = String(parameter.maximum);
    input.setAttribute("aria-label", parameter.unit ? `${parameter.name}，单位 ${parameter.unit}` : parameter.name);

    const help = document.createElement("small");
    const range = parameter.minimum != null || parameter.maximum != null
      ? `范围 ${parameter.minimum ?? "−∞"} ～ ${parameter.maximum ?? "+∞"}${parameter.unit ? ` ${parameter.unit}` : ""}`
      : "";
    help.textContent = [parameter.description, range].filter(Boolean).join("；") || `参数 ID：${parameter.id}`;
    label.append(title, input, help);
    return label;
  }

  function renderParameterFields(model) {
    elements.parameterFields.replaceChildren();
    const parameters = model?.parameters || [];
    elements.parameterEmpty.hidden = parameters.length > 0;
    elements.resetParameters.disabled = state.running || !modelHasConfig(model);

    for (const parameter of parameters) {
      elements.parameterFields.append(createNumericParameterField(parameter, { parameterId: parameter.id }));
    }
  }

  function renderModuleOptionDetails(slot, card) {
    const select = card.querySelector("select[data-module-slot-id]");
    const description = card.querySelector(".module-option-description");
    const parameterGrid = card.querySelector(".module-parameter-grid");
    const parameterEmpty = card.querySelector(".module-parameter-empty");
    const option = slot.options.find((item) => item.id === select.value) || slot.options[0];
    if (!option) return;
    select.value = option.id;
    description.textContent = option.description || `模块实现 ID：${option.id}`;
    parameterGrid.replaceChildren();
    parameterEmpty.hidden = option.parameters.length > 0;
    for (const parameter of option.parameters) {
      parameterGrid.append(createNumericParameterField(parameter, {
        moduleSlotId: slot.id,
        moduleOptionId: option.id,
        moduleParameterId: parameter.id,
      }));
    }
  }

  function renderModuleFields(model) {
    elements.moduleFields.replaceChildren();
    const modules = model?.modules || [];
    elements.moduleEmpty.hidden = modules.length > 0;
    if (!modules.length) {
      elements.moduleEmpty.querySelector("strong").textContent = model
        ? "当前模型使用内置模块"
        : "等待模块配置";
      elements.moduleEmpty.querySelector("p").textContent = model
        ? "服务端未为该模型声明可选模块；运行时继续使用模型内置的制导与控制配置。"
        : "选择模型后，可在此选择服务端为该模型开放的制导与控制实现。";
      return;
    }

    for (const slot of modules) {
      const card = document.createElement("article");
      card.className = "module-config-card";
      card.dataset.moduleSlotId = slot.id;

      const heading = document.createElement("div");
      heading.className = "module-card-heading";
      const headingText = document.createElement("div");
      const title = document.createElement("strong");
      const slotDescription = document.createElement("p");
      title.textContent = slot.name;
      slotDescription.textContent = slot.description || `配置槽位 ID：${slot.id}`;
      headingText.append(title, slotDescription);
      const slotCode = document.createElement("span");
      slotCode.textContent = slot.id;
      heading.append(headingText, slotCode);

      const body = document.createElement("div");
      body.className = "module-card-body";
      const optionField = document.createElement("label");
      optionField.className = "field module-option-field";
      const optionLabel = document.createElement("span");
      optionLabel.textContent = "实现方案";
      const select = document.createElement("select");
      select.required = true;
      select.dataset.moduleSlotId = slot.id;
      select.dataset.defaultOptionId = slot.defaultOptionId;
      select.setAttribute("aria-label", `${slot.name}实现方案`);
      select.replaceChildren(...slot.options.map((option) => new Option(option.name, option.id)));
      select.value = slot.defaultOptionId;
      const optionDescription = document.createElement("small");
      optionDescription.className = "module-option-description";
      optionField.append(optionLabel, select, optionDescription);

      const parameterPanel = document.createElement("div");
      parameterPanel.className = "module-parameter-panel";
      const parameterHeading = document.createElement("div");
      parameterHeading.className = "module-parameter-heading";
      const parameterTitle = document.createElement("strong");
      const parameterHint = document.createElement("span");
      parameterTitle.textContent = "当前方案参数";
      parameterHint.textContent = "切换方案后仅提交当前显示项";
      parameterHeading.append(parameterTitle, parameterHint);
      const parameterGrid = document.createElement("div");
      parameterGrid.className = "field-grid module-parameter-grid";
      const parameterEmpty = document.createElement("p");
      parameterEmpty.className = "module-parameter-empty";
      parameterEmpty.textContent = "该方案没有额外可配置参数。";
      parameterPanel.append(parameterHeading, parameterGrid, parameterEmpty);
      body.append(optionField, parameterPanel);
      card.append(heading, body);
      elements.moduleFields.append(card);

      select.addEventListener("change", () => renderModuleOptionDetails(slot, card));
      renderModuleOptionDetails(slot, card);
    }
  }

  function resetParameterValues() {
    elements.parameterFields.querySelectorAll("input[data-parameter-id]").forEach((input) => {
      input.value = input.defaultValue;
      input.setCustomValidity("");
    });
    const model = selectedModel();
    elements.moduleFields.querySelectorAll(".module-config-card").forEach((card) => {
      const slot = model?.modules?.find((item) => item.id === card.dataset.moduleSlotId);
      const select = card.querySelector("select[data-module-slot-id]");
      if (!slot || !select) return;
      select.value = slot.defaultOptionId;
      renderModuleOptionDetails(slot, card);
    });
    elements.formMessage.textContent = "模型参数和模块配置已恢复为服务端默认值。";
    elements.formMessage.classList.remove("is-error");
  }

  async function loadModels() {
    elements.reloadModels.disabled = true;
    elements.modelSelect.disabled = true;
    elements.runButton.disabled = true;
    elements.modelSelect.replaceChildren(new Option("正在加载模型…", ""));
    elements.modelHelp.textContent = "正在请求 GET /api/models";
    hideError();

    try {
      const { payload } = await apiRequest("/models");
      const models = modelItems(payload);
      if (!models.length) throw new Error("服务返回的模型列表为空。请先注册一个可运行模型。");
      state.models = models;
      const options = models.map((model) => {
        const option = new Option(model.available ? model.label : `${model.label}（不可用）`, model.id);
        option.disabled = !model.available;
        return option;
      });
      elements.modelSelect.replaceChildren(...options);
      const firstAvailable = options.find((option) => !option.disabled);
      if (!firstAvailable) throw new Error("模型列表中没有当前可运行的模型。 ");
      elements.modelSelect.value = firstAvailable.value;
      elements.modelSelect.disabled = false;
      elements.runButton.disabled = state.running;
      updateModelHelp();
      if (state.history.length) renderHistory();
    } catch (error) {
      setConnection(false, "连接失败");
      elements.modelSelect.replaceChildren(new Option("模型加载失败", ""));
      elements.modelHelp.textContent = "无法读取 /api/models";
      renderParameterFields(null);
      renderModuleFields(null);
      showError(errorMessage(error));
    } finally {
      elements.reloadModels.disabled = false;
    }
  }

  function buildRunPayload() {
    if (!elements.runForm.checkValidity()) {
      const invalid = elements.runForm.querySelector(":invalid");
      invalid?.focus();
      throw new Error("请检查标红的场景参数。所有字段都必须有效。 ");
    }
    if (!elements.modelSelect.value) throw new Error("请选择一个飞行器模型。");
    const name = elements.scenarioName.value.trim();
    if (!name) throw new Error("请输入场景名称。");
    const parameters = {};
    elements.parameterFields.querySelectorAll("input[data-parameter-id]").forEach((input) => {
      const value = Number(input.value);
      if (!Number.isFinite(value)) throw new Error(`参数“${input.getAttribute("aria-label") || input.dataset.parameterId}”必须是有效数字。`);
      parameters[input.dataset.parameterId] = value;
    });
    const modules = {};
    elements.moduleFields.querySelectorAll(".module-config-card").forEach((card) => {
      const select = card.querySelector("select[data-module-slot-id]");
      if (!select?.value) throw new Error("请选择每个制导与控制模块的实现方案。");
      const optionParameters = {};
      card.querySelectorAll("input[data-module-parameter-id]").forEach((input) => {
        const value = Number(input.value);
        if (!Number.isFinite(value)) {
          throw new Error(`模块参数“${input.getAttribute("aria-label") || input.dataset.moduleParameterId}”必须是有效数字。`);
        }
        optionParameters[input.dataset.moduleParameterId] = value;
      });
      modules[select.dataset.moduleSlotId] = { id: select.value, parameters: optionParameters };
    });
    const payload = { model_id: elements.modelSelect.value, name, parameters };
    if (Object.keys(modules).length) payload.modules = modules;
    return payload;
  }

  function extractRunId(payload, response) {
    const direct = payload && typeof payload === "object"
      ? payload.id ?? payload.run_id ?? payload.runId ?? payload.task_id
      : null;
    if (direct != null && String(direct)) return String(direct);
    const location = response.headers.get("Location");
    if (location) {
      const parts = location.replace(/\/+$/, "").split("/");
      if (parts.at(-1)) return decodeURIComponent(parts.at(-1));
    }
    throw new Error("服务已接受请求，但响应中缺少运行 ID。 ");
  }

  function setFormBusy(busy) {
    state.running = busy;
    elements.runForm.querySelectorAll("input, select, button[type='reset']").forEach((control) => {
      control.disabled = busy;
    });
    elements.runButton.disabled = busy || elements.modelSelect.disabled;
    elements.runButton.classList.toggle("is-busy", busy);
    elements.runButton.lastChild.textContent = busy ? " 正在运行" : " 启动仿真";
    elements.reloadModels.disabled = busy;
    elements.resetParameters.disabled = busy || !modelHasConfig(selectedModel());
    elements.cancelRun.hidden = !busy || !state.runId;
    elements.cancelRun.disabled = false;
    if (state.history.length) renderHistory();
  }

  function resetRunView() {
    stopPolling();
    state.runId = null;
    state.logs = [];
    state.logSet.clear();
    state.latestResult = null;
    elements.runId.textContent = "—";
    elements.runState.textContent = "正在创建";
    elements.simTime.textContent = "—";
    elements.updatedAt.textContent = "—";
    setProgress(null);
    elements.logOutput.textContent = "正在向服务端提交场景…";
    elements.resultsEmpty.hidden = false;
    elements.resultsContent.hidden = true;
    elements.resultActions.hidden = true;
    elements.resultsCaption.textContent = "任务完成后将在此显示时序和轨迹。";
  }

  async function startRun(event) {
    event.preventDefault();
    hideError();
    elements.formMessage.classList.remove("is-error");

    let payload;
    try {
      payload = buildRunPayload();
    } catch (error) {
      elements.formMessage.textContent = errorMessage(error);
      elements.formMessage.classList.add("is-error");
      return;
    }

    const token = ++state.runToken;
    resetRunView();
    setFormBusy(true);
    updateStatus("queued", "正在创建");
    elements.formMessage.textContent = "任务已提交，请等待服务端返回运行 ID。";
    showScreen("runtime");

    try {
      const { payload: responsePayload, response } = await apiRequest("/runs", {
        method: "POST",
        body: JSON.stringify(payload),
      });
      if (token !== state.runToken) return;
      state.runId = extractRunId(responsePayload, response);
      elements.runId.textContent = state.runId;
      elements.cancelRun.hidden = false;
      appendLogs([`运行 ${state.runId} 已创建。`]);
      elements.formMessage.textContent = `运行 ${state.runId} 正在执行。`;
      await pollRun(token);
    } catch (error) {
      if (token !== state.runToken) return;
      failRun(errorMessage(error));
    }
  }

  function normalizeState(payload) {
    const raw = payload && typeof payload === "object"
      ? payload.status ?? payload.state ?? payload.run_status ?? "running"
      : "running";
    return String(raw).trim().toLowerCase();
  }

  function statusLabel(status, payload = {}) {
    if (SUCCESS_STATES.has(status)) return "已完成";
    if (FAILURE_STATES.has(status)) return status.includes("cancel") ? "已取消" : "运行失败";
    if (QUEUED_STATES.has(status)) return "排队中";
    return payload.status_text || payload.message || "运行中";
  }

  function statusClass(status) {
    if (SUCCESS_STATES.has(status)) return "status-completed";
    if (FAILURE_STATES.has(status)) return "status-failed";
    if (QUEUED_STATES.has(status)) return "status-queued";
    return "status-running";
  }

  function updateStatus(status, label, payload = {}) {
    const text = label || statusLabel(status, payload);
    elements.runStatus.className = `status-badge ${statusClass(status)}`;
    elements.runStatus.textContent = text;
    elements.runState.textContent = text;
    elements.topRunState.textContent = text;
    elements.updatedAt.textContent = new Intl.DateTimeFormat("zh-CN", {
      hour: "2-digit",
      minute: "2-digit",
      second: "2-digit",
      hour12: false,
    }).format(new Date());
  }

  function progressValue(payload) {
    if (!payload || typeof payload !== "object") return null;
    let value = payload.progress ?? payload.progress_percent ?? payload.percent;
    if (value != null && Number.isFinite(Number(value))) {
      value = Number(value);
      if (value >= 0 && value <= 1) value *= 100;
      return Math.min(100, Math.max(0, value));
    }
    const completed = Number(payload.completed_steps ?? payload.current_step);
    const total = Number(payload.total_steps ?? payload.step_count);
    if (Number.isFinite(completed) && Number.isFinite(total) && total > 0) {
      return Math.min(100, Math.max(0, completed / total * 100));
    }
    const simTime = Number(payload.sim_time ?? payload.simulation_time ?? payload.time);
    const maxTime = Number(payload.max_time ?? payload.duration);
    if (Number.isFinite(simTime) && Number.isFinite(maxTime) && maxTime > 0) {
      return Math.min(100, Math.max(0, simTime / maxTime * 100));
    }
    return null;
  }

  function setProgress(value, completed = false) {
    const numeric = completed ? 100 : value;
    if (numeric == null || !Number.isFinite(numeric)) {
      elements.progressTrack.classList.toggle("is-indeterminate", state.running);
      elements.progressTrack.removeAttribute("aria-valuenow");
      elements.progressBar.style.width = state.running ? "32%" : "0%";
      elements.progressLabel.textContent = state.running ? "运行中" : "0%";
      return;
    }
    const bounded = Math.min(100, Math.max(0, numeric));
    elements.progressTrack.classList.remove("is-indeterminate");
    elements.progressTrack.setAttribute("aria-valuenow", String(Math.round(bounded)));
    elements.progressBar.style.width = `${bounded}%`;
    elements.progressLabel.textContent = `${Math.round(bounded)}%`;
  }

  function normalizeLogs(value) {
    if (value == null) return [];
    const source = Array.isArray(value) ? value : [value];
    return source.map((entry) => {
      if (typeof entry === "string") return entry;
      if (entry && typeof entry === "object") {
        const time = entry.time ?? entry.timestamp ?? entry.at;
        const level = entry.level ?? entry.severity;
        const message = entry.message ?? entry.text ?? entry.detail ?? JSON.stringify(entry);
        return [time, level, message].filter((part) => part != null && part !== "").join(" · ");
      }
      return String(entry);
    });
  }

  function appendLogs(logs) {
    for (const line of normalizeLogs(logs)) {
      if (!state.logSet.has(line)) {
        state.logs.push(line);
        state.logSet.add(line);
      }
    }
    if (state.logs.length > 1000) {
      state.logs.splice(0, state.logs.length - 1000);
      state.logSet = new Set(state.logs);
    }
    elements.logOutput.textContent = state.logs.length ? state.logs.join("\n") : "暂无日志。";
    elements.logOutput.scrollTop = elements.logOutput.scrollHeight;
  }

  function updateRunPayload(payload) {
    const status = normalizeState(payload);
    updateStatus(status, null, payload || {});
    setProgress(progressValue(payload), SUCCESS_STATES.has(status));
    const simTime = payload && (payload.sim_time ?? payload.simulation_time ?? payload.time_s ?? payload.time);
    if (simTime != null && Number.isFinite(Number(simTime))) {
      elements.simTime.textContent = `${formatNumber(Number(simTime))} s`;
    }
    appendLogs(payload && (payload.logs ?? payload.log ?? payload.messages ?? payload.events));
    const backendError = payload && (payload.error ?? payload.error_message ?? payload.detail);
    if (backendError && FAILURE_STATES.has(status)) appendLogs([`错误：${errorMessage(backendError)}`]);
    return status;
  }

  async function pollRun(token) {
    if (!state.runId || token !== state.runToken) return;
    try {
      const { payload } = await apiRequest(`/runs/${encodeURIComponent(state.runId)}`);
      if (token !== state.runToken) return;
      state.pollErrors = 0;
      const status = updateRunPayload(payload || {});
      if (SUCCESS_STATES.has(status)) {
        if (payload && payload.result_available === false) {
          appendLogs(["运行已完成，等待结果文件可用…"]);
          schedulePoll(token, 500);
          return;
        }
        await loadResult(token);
        return;
      }
      if (FAILURE_STATES.has(status)) {
        const detail = payload && (payload.error ?? payload.error_message ?? payload.detail ?? payload.message);
        failRun(detail ? errorMessage(detail) : `运行 ${state.runId} 未成功完成。`);
        return;
      }
      schedulePoll(token, POLL_INTERVAL_MS);
    } catch (error) {
      if (token !== state.runToken) return;
      state.pollErrors += 1;
      appendLogs([`状态查询失败（${state.pollErrors}/${MAX_POLL_ERRORS}）：${errorMessage(error)}`]);
      if (state.pollErrors >= MAX_POLL_ERRORS) {
        failRun("连续多次无法读取运行状态。任务可能仍在服务端执行，请检查服务日志。");
        return;
      }
      setConnection(false, "连接不稳定");
      schedulePoll(token, Math.min(5000, POLL_INTERVAL_MS * (state.pollErrors + 1)));
    }
  }

  function schedulePoll(token, delay) {
    stopPolling();
    state.pollTimer = window.setTimeout(() => pollRun(token), delay);
  }

  function stopPolling() {
    if (state.pollTimer != null) {
      window.clearTimeout(state.pollTimer);
      state.pollTimer = null;
    }
  }

  async function loadResult(token) {
    appendLogs(["运行完成，正在加载结果数据…"]);
    try {
      const { payload } = await apiRequest(`/runs/${encodeURIComponent(state.runId)}/result`);
      if (token !== state.runToken) return;
      if (payload == null || typeof payload !== "object") {
        throw new Error("结果接口没有返回可绘制的 JSON 数据。 ");
      }
      state.latestResult = payload;
      state.resultCache.set(state.runId, payload);
      appendLogs(payload.logs ?? payload.log ?? payload.messages);
      renderResult(payload);
      updateResultActions(state.runId);
      updateStatus("completed", "已完成");
      setProgress(100, true);
      elements.formMessage.textContent = `运行 ${state.runId} 已完成，结果已加载。`;
      setFormBusy(false);
      showScreen("results");
      void loadHistory({ silent: true });
    } catch (error) {
      failRun(`运行已完成，但结果加载失败：${errorMessage(error)}`);
    }
  }

  function updateResultActions(runId) {
    const encoded = encodeURIComponent(runId);
    elements.downloadResultJson.href = `${API_BASE}/runs/${encoded}/export?format=json`;
    elements.downloadResultCsv.href = `${API_BASE}/runs/${encoded}/export?format=csv`;
    elements.downloadResultRaw.href = `${API_BASE}/runs/${encoded}/export?format=raw`;
    elements.resultActions.hidden = false;
  }

  function failRun(message) {
    stopPolling();
    updateStatus("failed", "运行失败");
    appendLogs([`错误：${message}`]);
    elements.formMessage.textContent = message;
    elements.formMessage.classList.add("is-error");
    setFormBusy(false);
    showError(message);
    void loadHistory({ silent: true });
  }

  async function cancelRun() {
    if (!state.runId || !state.running) return;
    elements.cancelRun.disabled = true;
    appendLogs([`正在请求取消运行 ${state.runId}…`]);
    try {
      await apiRequest(`/runs/${encodeURIComponent(state.runId)}/cancel`, { method: "POST" });
      updateStatus("running", "正在取消");
      appendLogs(["取消请求已提交，等待服务端确认。"]);
    } catch (error) {
      elements.cancelRun.disabled = false;
      showError(`取消运行失败：${errorMessage(error)}`);
      appendLogs([`取消请求失败：${errorMessage(error)}`]);
    }
  }

  function updateModelHelp() {
    const model = selectedModel();
    elements.modelHelp.textContent = model?.description || `模型 ID：${elements.modelSelect.value || "—"}`;
    const generatedName = `${model?.name || model?.id || "未命名模型"} 基准场景`;
    if (!elements.scenarioName.value.trim() || elements.scenarioName.value === state.generatedScenarioName) {
      elements.scenarioName.value = generatedName;
      state.generatedScenarioName = generatedName;
    }
    renderParameterFields(model);
    renderModuleFields(model);
  }

  function historyItems(payload) {
    const source = Array.isArray(payload) ? payload : payload && (payload.runs || payload.items || payload.data);
    if (!Array.isArray(source)) return [];
    return source.map((item) => {
      if (!item || typeof item !== "object") return null;
      const runId = item.run_id ?? item.id ?? item.runId;
      if (runId == null || !String(runId)) return null;
      const status = normalizeState(item);
      return {
        ...item,
        run_id: String(runId),
        model_id: String(item.model_id ?? item.model ?? "—"),
        model_name: String(item.model_name ?? item.model_label ?? item.model_id ?? item.model ?? "—"),
        name: String(item.name ?? item.scenario_name ?? `运行 ${String(runId).slice(0, 8)}`),
        parameters: item.parameters && typeof item.parameters === "object" ? item.parameters : {},
        modules: runModuleItems(item.modules),
        status,
        result_available: item.result_available === true || (item.result_available !== false && SUCCESS_STATES.has(status)),
      };
    }).filter(Boolean);
  }

  function formatDateTime(value) {
    if (!value) return "—";
    const date = new Date(value);
    if (Number.isNaN(date.getTime())) return String(value);
    return new Intl.DateTimeFormat("zh-CN", {
      month: "2-digit", day: "2-digit", hour: "2-digit", minute: "2-digit", second: "2-digit", hour12: false,
    }).format(date);
  }

  function formatParameters(parameters, limit = 4, emptyLabel = "默认参数") {
    const entries = Object.entries(parameters || {});
    if (!entries.length) return emptyLabel;
    return entries.slice(0, limit).map(([key, value]) => `${key}=${typeof value === "number" ? formatNumber(value) : value}`).join(" · ")
      + (entries.length > limit ? ` · +${entries.length - limit}` : "");
  }

  function moduleDisplayInfo(selection, modelId) {
    const model = state.models.find((item) => item.id === modelId);
    const slot = model?.modules?.find((item) => item.id === selection.slot_id);
    const option = slot?.options?.find((item) => item.id === selection.id);
    const fallbackSlotNames = { guidance: "制导", control: "控制" };
    return {
      slotName: selection.slot_name || slot?.name || fallbackSlotNames[selection.slot_id] || selection.slot_id,
      optionName: selection.name || option?.name || selection.id,
      option,
    };
  }

  function formatModuleParameters(selection, option, limit = Number.POSITIVE_INFINITY) {
    const entries = Object.entries(selection.parameters || {});
    if (!entries.length) return "";
    const definitions = new Map((option?.parameters || []).map((parameter) => [parameter.id, parameter]));
    return entries.slice(0, limit).map(([key, value]) => {
      const definition = definitions.get(key);
      const label = definition?.name || key;
      const formatted = typeof value === "number" ? formatNumber(value) : value;
      return `${label}=${formatted}${definition?.unit ? ` ${definition.unit}` : ""}`;
    }).join(" · ") + (entries.length > limit ? ` · +${entries.length - limit}` : "");
  }

  function formatModules(modules, modelId, parameterLimit = Number.POSITIVE_INFINITY) {
    const selections = Array.isArray(modules) ? modules : runModuleItems(modules);
    return selections.map((selection) => {
      const info = moduleDisplayInfo(selection, modelId);
      const parameterText = formatModuleParameters(selection, info.option, parameterLimit);
      return `${info.slotName}=${info.optionName}${parameterText ? `（${parameterText}）` : ""}`;
    }).join("；");
  }

  function formatRunConfiguration(run, compact = true) {
    const moduleText = formatModules(run.modules, run.model_id, compact ? 3 : Number.POSITIVE_INFINITY);
    const scenarioText = formatParameters(
      run.parameters,
      compact ? 4 : Number.POSITIVE_INFINITY,
      "默认场景参数",
    );
    return moduleText ? `${moduleText}；场景：${scenarioText}` : scenarioText;
  }

  function exportLink(runId, format, label) {
    const anchor = document.createElement("a");
    anchor.className = "history-action-link";
    anchor.href = `${API_BASE}/runs/${encodeURIComponent(runId)}/export?format=${format}`;
    anchor.textContent = label;
    return anchor;
  }

  function successfulHistoryRun(run) {
    return SUCCESS_STATES.has(run.status) && run.result_available;
  }

  function selectedComparisonRuns() {
    return [...state.selectedCompareIds]
      .map((runId) => state.history.find((run) => run.run_id === runId))
      .filter(Boolean);
  }

  function clearComparisonView(message) {
    elements.comparisonContent.hidden = true;
    elements.comparisonEmpty.hidden = false;
    elements.comparisonEmpty.querySelector("strong").textContent = "尚未选择可比较运行";
    elements.comparisonEmpty.querySelector("p").textContent = "历史记录中的比较复选框会控制此处的运行集合。";
    elements.comparisonCaption.textContent = message || "请先在运行历史中选择 2–4 条同模型成功运行；模块组合可以不同。";
  }

  function syncComparisonControls() {
    const availableIds = new Set(state.history.filter(successfulHistoryRun).map((run) => run.run_id));
    for (const runId of state.selectedCompareIds) {
      if (!availableIds.has(runId)) state.selectedCompareIds.delete(runId);
    }
    const selected = selectedComparisonRuns();
    const sameModel = selected.length < 2 || selected.every((run) => run.model_id === selected[0].model_id);
    const valid = selected.length >= 2 && selected.length <= 4 && sameModel;
    elements.compareSelectionCount.textContent = `已选 ${selected.length} / 4`;
    elements.compareRuns.disabled = state.comparisonRunning || !valid;
    if (!elements.comparisonContent.hidden && valid) return;
    if (!selected.length) {
      elements.comparisonCaption.textContent = "请先在运行历史中选择 2–4 条同模型成功运行；模块组合可以不同。";
    } else if (selected.length === 1) {
      elements.comparisonCaption.textContent = "还需选择至少 1 条同模型成功运行。";
    } else if (!sameModel) {
      elements.comparisonCaption.textContent = "只能比较同一模型产生的运行结果。";
    } else {
      elements.comparisonCaption.textContent = `已选择 ${selected.length} 条 ${selected[0].model_name} 运行，可比较不同模块组合。`;
    }
  }

  function toggleComparisonSelection(run, checkbox) {
    if (!checkbox.checked) {
      state.selectedCompareIds.delete(run.run_id);
      clearComparisonView();
      syncComparisonControls();
      return;
    }
    const selected = selectedComparisonRuns();
    if (selected.length >= 4) {
      checkbox.checked = false;
      elements.comparisonCaption.textContent = "一次最多比较 4 条运行。";
      return;
    }
    if (selected.length && selected[0].model_id !== run.model_id) {
      checkbox.checked = false;
      elements.comparisonCaption.textContent = `已选择 ${selected[0].model_name}，不能再选择 ${run.model_name}。`;
      return;
    }
    state.selectedCompareIds.add(run.run_id);
    clearComparisonView();
    syncComparisonControls();
  }

  function renderHistory() {
    elements.runHistoryBody.replaceChildren();
    elements.historyEmpty.hidden = state.history.length > 0;
    elements.historyContent.hidden = state.history.length === 0;

    for (const run of state.history) {
      const row = document.createElement("tr");

      const selectCell = document.createElement("td");
      const checkbox = document.createElement("input");
      checkbox.type = "checkbox";
      checkbox.className = "comparison-checkbox";
      checkbox.checked = state.selectedCompareIds.has(run.run_id);
      checkbox.disabled = !successfulHistoryRun(run);
      checkbox.setAttribute("aria-label", `选择 ${run.name} 进行比较`);
      checkbox.addEventListener("change", () => toggleComparisonSelection(run, checkbox));
      selectCell.append(checkbox);

      const nameCell = document.createElement("td");
      const name = document.createElement("strong");
      const runCode = document.createElement("small");
      name.textContent = run.name;
      runCode.textContent = run.run_id;
      nameCell.append(name, runCode);

      const modelCell = document.createElement("td");
      modelCell.textContent = run.model_name;

      const parameterCell = document.createElement("td");
      parameterCell.className = "history-parameters";
      const moduleText = formatModules(run.modules, run.model_id, 3);
      if (moduleText) {
        const moduleLine = document.createElement("strong");
        const scenarioLine = document.createElement("small");
        moduleLine.textContent = moduleText;
        scenarioLine.textContent = `场景：${formatParameters(run.parameters, 4, "默认参数")}`;
        parameterCell.append(moduleLine, scenarioLine);
      } else {
        parameterCell.textContent = formatParameters(run.parameters);
      }
      parameterCell.title = formatRunConfiguration(run, false);

      const statusCell = document.createElement("td");
      const badge = document.createElement("span");
      badge.className = `status-badge ${statusClass(run.status)}`;
      badge.textContent = statusLabel(run.status, run);
      statusCell.append(badge);

      const timeCell = document.createElement("td");
      timeCell.textContent = formatDateTime(run.created_at ?? run.started_at);

      const actionsCell = document.createElement("td");
      actionsCell.className = "row-actions";
      const viewButton = document.createElement("button");
      viewButton.type = "button";
      viewButton.className = "history-action-button";
      viewButton.textContent = "查看";
      viewButton.disabled = state.running || !run.result_available;
      viewButton.addEventListener("click", () => void openHistoricalRun(run));
      actionsCell.append(viewButton);
      if (run.result_available) {
        actionsCell.append(exportLink(run.run_id, "json", "JSON"));
        actionsCell.append(exportLink(run.run_id, "csv", "CSV"));
        actionsCell.append(exportLink(run.run_id, "raw", "Raw"));
      }

      row.append(selectCell, nameCell, modelCell, parameterCell, statusCell, timeCell, actionsCell);
      elements.runHistoryBody.append(row);
    }
    syncComparisonControls();
  }

  async function loadHistory({ silent = false } = {}) {
    elements.reloadHistory.disabled = true;
    try {
      const { payload } = await apiRequest("/runs");
      state.history = historyItems(payload);
      renderHistory();
      if (!state.history.length) {
        elements.historyEmpty.querySelector("strong").textContent = "暂无运行记录";
        elements.historyEmpty.querySelector("p").textContent = "完成一次仿真后，记录会显示在这里。";
      }
    } catch (error) {
      elements.historyContent.hidden = true;
      elements.historyEmpty.hidden = false;
      elements.historyEmpty.querySelector("strong").textContent = "历史记录加载失败";
      elements.historyEmpty.querySelector("p").textContent = errorMessage(error);
      if (!silent) showError(`加载运行历史失败：${errorMessage(error)}`);
    } finally {
      elements.reloadHistory.disabled = false;
    }
  }

  async function openHistoricalRun(run) {
    if (state.running || !run.result_available) return;
    hideError();
    elements.formMessage.classList.remove("is-error");
    elements.formMessage.textContent = `正在加载历史运行 ${run.run_id}…`;
    try {
      const detailPromise = apiRequest(`/runs/${encodeURIComponent(run.run_id)}`);
      const resultPromise = state.resultCache.has(run.run_id)
        ? Promise.resolve({ payload: state.resultCache.get(run.run_id) })
        : apiRequest(`/runs/${encodeURIComponent(run.run_id)}/result`);
      const [{ payload: detail }, { payload: result }] = await Promise.all([detailPromise, resultPromise]);
      if (!result || typeof result !== "object") throw new Error("历史结果不是有效 JSON 数据。");
      state.runId = run.run_id;
      state.latestResult = result;
      state.resultCache.set(run.run_id, result);
      state.logs = [];
      state.logSet.clear();
      elements.logOutput.textContent = "暂无日志。";
      elements.runId.textContent = run.run_id;
      updateRunPayload(detail || run);
      renderResult(result);
      updateResultActions(run.run_id);
      elements.formMessage.textContent = `已载入历史运行“${run.name}”。`;
      showScreen("results");
    } catch (error) {
      const message = `加载历史运行失败：${errorMessage(error)}`;
      elements.formMessage.textContent = message;
      elements.formMessage.classList.add("is-error");
      showError(message);
    }
  }

  const FIELD_ALIASES = {
    time: ["time", "t", "time_s", "fly_time", "flytime", "simulation_time"],
    altitude: ["altitude", "altitude_m", "alt", "height", "height_m", "up", "up_m", "lla_alt"],
    speed: ["speed", "speed_m_s", "speed_mps", "velocity", "velocity_m_s", "v"],
    east: ["east", "east_m", "x", "x_m", "position_east", "position_east_m"],
    north: ["north", "north_m", "y", "y_m", "position_north", "position_north_m"],
    longitude: ["longitude", "lon", "lng", "longitude_deg"],
    latitude: ["latitude", "lat", "latitude_deg"],
  };

  function normalizedKey(value) {
    return String(value).trim().toLowerCase().replace(/[^a-z0-9]+/g, "_").replace(/^_|_$/g, "");
  }

  function valueFor(sample, field) {
    if (!sample || typeof sample !== "object") return undefined;
    const index = new Map(Object.keys(sample).map((key) => [normalizedKey(key), key]));
    for (const alias of FIELD_ALIASES[field]) {
      const key = index.get(alias);
      if (key != null) return sample[key];
    }
    return undefined;
  }

  function samplesFromRows(container) {
    const columns = container && (container.columns || container.fields || container.headers);
    const rows = container && (container.rows || container.values);
    if (!Array.isArray(columns) || !Array.isArray(rows)) return [];
    return rows.map((row) => {
      const sample = {};
      columns.forEach((column, index) => { sample[column] = Array.isArray(row) ? row[index] : undefined; });
      return sample;
    });
  }

  function samplesFromColumns(container) {
    if (!container || typeof container !== "object" || Array.isArray(container)) return [];
    const arrays = Object.entries(container).filter(([, value]) => Array.isArray(value));
    if (!arrays.length) return [];
    const length = Math.max(...arrays.map(([, value]) => value.length));
    if (!length) return [];
    return Array.from({ length }, (_, row) => Object.fromEntries(arrays.map(([key, values]) => [key, values[row]])));
  }

  function extractSamples(result) {
    const candidates = [
      result.samples,
      result.trajectory,
      result.timeseries,
      result.series,
      result.data,
      result.result,
      result,
    ].filter(Boolean);
    for (const candidate of candidates) {
      if (Array.isArray(candidate) && candidate.some((item) => item && typeof item === "object")) {
        return candidate;
      }
      const rows = samplesFromRows(candidate);
      if (rows.length) return rows;
      const columns = samplesFromColumns(candidate);
      if (columns.length && columns.some((sample) => valueFor(sample, "time") != null)) return columns;
    }
    return [];
  }

  function numericSeries(samples, field) {
    return samples.map((sample) => Number(valueFor(sample, field)));
  }

  function finitePairs(xs, ys) {
    const count = Math.min(xs.length, ys.length);
    const points = [];
    for (let i = 0; i < count; i += 1) {
      if (Number.isFinite(xs[i]) && Number.isFinite(ys[i])) points.push({ x: xs[i], y: ys[i] });
    }
    return points;
  }

  function svgElement(name, attributes = {}, text = null) {
    const node = document.createElementNS(SVG_NS, name);
    for (const [key, value] of Object.entries(attributes)) node.setAttribute(key, String(value));
    if (text != null) node.textContent = text;
    return node;
  }

  function formatNumber(value) {
    if (!Number.isFinite(value)) return "—";
    const absolute = Math.abs(value);
    if ((absolute > 0 && absolute < 0.01) || absolute >= 1e7) return value.toExponential(2);
    return new Intl.NumberFormat("zh-CN", { maximumFractionDigits: absolute < 10 ? 3 : 2 }).format(value);
  }

  function extent(values) {
    let minimum = Math.min(...values);
    let maximum = Math.max(...values);
    if (minimum === maximum) {
      const pad = Math.abs(minimum) * 0.05 || 1;
      minimum -= pad;
      maximum += pad;
    }
    return [minimum, maximum];
  }

  function renderLineChart(svg, points, options) {
    svg.replaceChildren();
    const width = 720;
    const height = options.height || 320;
    const margin = { top: 24, right: 24, bottom: 52, left: 72 };
    const plotWidth = width - margin.left - margin.right;
    const plotHeight = height - margin.top - margin.bottom;

    if (points.length < 2) {
      svg.append(svgElement("text", { x: width / 2, y: height / 2, "text-anchor": "middle", class: "empty-chart-text" }, "结果中没有足够的可绘制数据"));
      return;
    }

    const [xMin, xMax] = extent(points.map((point) => point.x));
    const [yMin, yMax] = extent(points.map((point) => point.y));
    const sx = (value) => margin.left + (value - xMin) / (xMax - xMin) * plotWidth;
    const sy = (value) => margin.top + plotHeight - (value - yMin) / (yMax - yMin) * plotHeight;
    const ticks = 5;

    for (let index = 0; index <= ticks; index += 1) {
      const ratio = index / ticks;
      const x = margin.left + ratio * plotWidth;
      const y = margin.top + ratio * plotHeight;
      svg.append(svgElement("line", { x1: x, y1: margin.top, x2: x, y2: margin.top + plotHeight, class: "grid-line" }));
      svg.append(svgElement("line", { x1: margin.left, y1: y, x2: margin.left + plotWidth, y2: y, class: "grid-line" }));
      svg.append(svgElement("text", { x, y: height - 26, "text-anchor": "middle", class: "axis-text" }, formatNumber(xMin + ratio * (xMax - xMin))));
      svg.append(svgElement("text", { x: margin.left - 10, y: y + 3, "text-anchor": "end", class: "axis-text" }, formatNumber(yMax - ratio * (yMax - yMin))));
    }

    svg.append(svgElement("line", { x1: margin.left, y1: margin.top + plotHeight, x2: margin.left + plotWidth, y2: margin.top + plotHeight, class: "axis-line" }));
    svg.append(svgElement("line", { x1: margin.left, y1: margin.top, x2: margin.left, y2: margin.top + plotHeight, class: "axis-line" }));
    svg.append(svgElement("text", { x: margin.left + plotWidth / 2, y: height - 7, "text-anchor": "middle", class: "axis-title" }, options.xLabel));
    const yTitle = svgElement("text", { x: 15, y: margin.top + plotHeight / 2, "text-anchor": "middle", class: "axis-title", transform: `rotate(-90 15 ${margin.top + plotHeight / 2})` }, options.yLabel);
    svg.append(yTitle);

    const maxPoints = 3000;
    const step = Math.max(1, Math.ceil(points.length / maxPoints));
    const rendered = points.filter((_, index) => index % step === 0 || index === points.length - 1);
    const path = rendered.map((point, index) => `${index ? "L" : "M"}${sx(point.x).toFixed(2)},${sy(point.y).toFixed(2)}`).join(" ");
    svg.append(svgElement("path", { d: path, class: `series-line ${options.seriesClass}` }));

    if (options.endpoints) {
      const first = rendered[0];
      const last = rendered.at(-1);
      svg.append(svgElement("circle", { cx: sx(first.x), cy: sy(first.y), r: 5, class: "chart-start" }));
      svg.append(svgElement("circle", { cx: sx(last.x), cy: sy(last.y), r: 5, class: "chart-end" }));
    }
  }

  function renderOverlayChart(svg, seriesList, options) {
    svg.replaceChildren();
    const width = 720;
    const height = options.height || 320;
    const margin = { top: 24, right: 24, bottom: 52, left: 72 };
    const plotWidth = width - margin.left - margin.right;
    const plotHeight = height - margin.top - margin.bottom;
    const usable = seriesList.filter((series) => Array.isArray(series.points) && series.points.length >= 2);

    if (!usable.length) {
      svg.append(svgElement("text", { x: width / 2, y: height / 2, "text-anchor": "middle", class: "empty-chart-text" }, "比较结果中没有足够的可绘制数据"));
      return;
    }

    const allPoints = usable.flatMap((series) => series.points);
    const [xMin, xMax] = extent(allPoints.map((point) => point.x));
    const [yMin, yMax] = extent(allPoints.map((point) => point.y));
    const sx = (value) => margin.left + (value - xMin) / (xMax - xMin) * plotWidth;
    const sy = (value) => margin.top + plotHeight - (value - yMin) / (yMax - yMin) * plotHeight;
    const ticks = 5;

    for (let index = 0; index <= ticks; index += 1) {
      const ratio = index / ticks;
      const x = margin.left + ratio * plotWidth;
      const y = margin.top + ratio * plotHeight;
      svg.append(svgElement("line", { x1: x, y1: margin.top, x2: x, y2: margin.top + plotHeight, class: "grid-line" }));
      svg.append(svgElement("line", { x1: margin.left, y1: y, x2: margin.left + plotWidth, y2: y, class: "grid-line" }));
      svg.append(svgElement("text", { x, y: height - 26, "text-anchor": "middle", class: "axis-text" }, formatNumber(xMin + ratio * (xMax - xMin))));
      svg.append(svgElement("text", { x: margin.left - 10, y: y + 3, "text-anchor": "end", class: "axis-text" }, formatNumber(yMax - ratio * (yMax - yMin))));
    }

    svg.append(svgElement("line", { x1: margin.left, y1: margin.top + plotHeight, x2: margin.left + plotWidth, y2: margin.top + plotHeight, class: "axis-line" }));
    svg.append(svgElement("line", { x1: margin.left, y1: margin.top, x2: margin.left, y2: margin.top + plotHeight, class: "axis-line" }));
    svg.append(svgElement("text", { x: margin.left + plotWidth / 2, y: height - 7, "text-anchor": "middle", class: "axis-title" }, options.xLabel));
    svg.append(svgElement("text", {
      x: 15,
      y: margin.top + plotHeight / 2,
      "text-anchor": "middle",
      class: "axis-title",
      transform: `rotate(-90 15 ${margin.top + plotHeight / 2})`,
    }, options.yLabel));

    for (const [index, series] of usable.entries()) {
      const step = Math.max(1, Math.ceil(series.points.length / 3000));
      const rendered = series.points.filter((_, pointIndex) => pointIndex % step === 0 || pointIndex === series.points.length - 1);
      const path = rendered.map((point, pointIndex) => `${pointIndex ? "L" : "M"}${sx(point.x).toFixed(2)},${sy(point.y).toFixed(2)}`).join(" ");
      const attributes = {
        d: path,
        class: "series-line comparison-series",
        stroke: series.color || COMPARISON_COLORS[index % COMPARISON_COLORS.length],
      };
      if (series.dash) attributes["stroke-dasharray"] = series.dash;
      svg.append(svgElement("path", attributes));
    }
  }

  const SUMMARY_LABELS = {
    duration: "仿真时长",
    duration_s: "仿真时长",
    sample_count: "样本数",
    returned_sample_count: "返回样本数",
    max_altitude: "最大高度",
    max_altitude_m: "最大高度",
    max_speed: "最大速度",
    max_speed_m_s: "最大速度",
    final_distance: "最终距离",
    final_distance_m: "最终距离",
    closest_distance: "最小脱靶量",
    closest_distance_m: "最小脱靶量",
    miss_distance: "脱靶量",
    miss_distance_m: "脱靶量",
    flight_time: "飞行时间",
    flight_time_s: "飞行时间",
    final_time_s: "最终时刻",
    final_north_m: "最终北向位置",
    final_east_m: "最终东向位置",
    final_up_m: "最终天向位置",
    final_speed_m_s: "最终速度",
    final_altitude_m: "最终高度",
    stop_reason: "停止原因",
    termination_reason: "终止原因",
    model: "模型",
    status: "状态",
  };

  function summaryLabel(key) {
    const normalized = normalizedKey(key);
    return SUMMARY_LABELS[normalized] || String(key).replace(/_/g, " ");
  }

  function summaryValue(key, value) {
    if (value == null) return "—";
    if (typeof value === "number") {
      const normalized = normalizedKey(key);
      const unit = normalized.endsWith("_m_s") ? " m/s"
        : normalized.endsWith("_s") || normalized.includes("time") || normalized === "duration" ? " s"
          : normalized.endsWith("_m") || normalized.includes("altitude") || normalized.includes("distance") ? " m"
            : "";
      return `${formatNumber(value)}${unit}`;
    }
    if (typeof value === "object") return JSON.stringify(value);
    return String(value);
  }

  function derivedSummary(samples, series) {
    const times = series.time.filter(Number.isFinite);
    const altitude = series.altitude.filter(Number.isFinite);
    const speed = series.speed.filter(Number.isFinite);
    const summary = { sample_count: samples.length };
    if (times.length) summary.duration_s = Math.max(...times) - Math.min(...times);
    if (altitude.length) summary.max_altitude_m = Math.max(...altitude);
    if (speed.length) summary.max_speed_m_s = Math.max(...speed);
    return summary;
  }

  function renderSummary(summary) {
    elements.summaryGrid.replaceChildren();
    const entries = Object.entries(summary || {}).filter(([, value]) => value != null).slice(0, 12);
    if (!entries.length) entries.push(["status", "已完成"]);
    for (const [key, value] of entries) {
      const item = document.createElement("div");
      const term = document.createElement("dt");
      const description = document.createElement("dd");
      term.textContent = summaryLabel(key);
      description.textContent = summaryValue(key, value);
      item.append(term, description);
      elements.summaryGrid.append(item);
    }
  }

  function comparisonRunItems(payload) {
    let source = Array.isArray(payload) ? payload : payload && payload.runs;
    if (!Array.isArray(source)) return [];
    const selectedOrder = new Map([...state.selectedCompareIds].map((runId, index) => [runId, index]));
    source = [...source].sort((left, right) => {
      const leftId = String(left?.run_id ?? left?.id ?? "");
      const rightId = String(right?.run_id ?? right?.id ?? "");
      return (selectedOrder.get(leftId) ?? Number.MAX_SAFE_INTEGER) - (selectedOrder.get(rightId) ?? Number.MAX_SAFE_INTEGER);
    });
    return source.map((item, index) => {
      if (!item || typeof item !== "object") return null;
      const runId = String(item.run_id ?? item.id ?? "");
      const historyRun = state.history.find((run) => run.run_id === runId);
      const samples = extractSamples(item);
      const series = {
        time: numericSeries(samples, "time"),
        altitude: numericSeries(samples, "altitude"),
        speed: numericSeries(samples, "speed"),
      };
      return {
        ...item,
        run_id: runId,
        name: String(item.name ?? historyRun?.name ?? `运行 ${runId.slice(0, 8) || index + 1}`),
        model_id: String(item.model_id ?? historyRun?.model_id ?? ""),
        parameters: item.parameters && typeof item.parameters === "object" ? item.parameters : historyRun?.parameters || {},
        modules: item.modules != null ? runModuleItems(item.modules) : historyRun?.modules || [],
        summary: { ...derivedSummary(samples, series), ...(item.summary && typeof item.summary === "object" ? item.summary : {}) },
        series,
        color: COMPARISON_COLORS[index % COMPARISON_COLORS.length],
        dash: index === 0 ? "" : ["9 5", "3 4", "12 4 3 4"][index - 1],
      };
    }).filter(Boolean);
  }

  function comparisonMetricKeys(runs) {
    const keys = new Set(runs.flatMap((run) => Object.keys(run.summary || {})));
    const preferred = [
      "duration_s", "final_time_s", "max_altitude_m", "max_speed_m_s", "final_altitude_m",
      "final_speed_m_s", "final_north_m", "final_east_m", "final_up_m", "sample_count", "returned_sample_count",
    ];
    return [...keys]
      .filter((key) => runs.some((run) => run.summary?.[key] != null && typeof run.summary[key] !== "object"))
      .sort((a, b) => {
        const ai = preferred.indexOf(a);
        const bi = preferred.indexOf(b);
        return (ai < 0 ? preferred.length : ai) - (bi < 0 ? preferred.length : bi) || a.localeCompare(b);
      })
      .slice(0, 12);
  }

  function renderComparisonMetrics(runs) {
    const headRow = document.createElement("tr");
    const metricHeading = document.createElement("th");
    metricHeading.scope = "col";
    metricHeading.textContent = "指标";
    headRow.append(metricHeading);
    for (const [index, run] of runs.entries()) {
      const heading = document.createElement("th");
      heading.scope = "col";
      const marker = document.createElement("span");
      marker.className = "legend-swatch compact";
      marker.style.background = run.color;
      const label = document.createElement("span");
      label.textContent = index === 0 ? `${run.name}（基准）` : run.name;
      heading.append(marker, label);
      headRow.append(heading);
    }
    elements.comparisonMetricsHead.replaceChildren(headRow);
    elements.comparisonMetricsBody.replaceChildren();

    const keys = comparisonMetricKeys(runs);
    if (!keys.length) {
      const row = document.createElement("tr");
      const cell = document.createElement("td");
      cell.colSpan = runs.length + 1;
      cell.textContent = "比较结果未返回可展示的摘要指标。";
      row.append(cell);
      elements.comparisonMetricsBody.append(row);
      return;
    }

    for (const key of keys) {
      const row = document.createElement("tr");
      const labelCell = document.createElement("th");
      labelCell.scope = "row";
      labelCell.textContent = summaryLabel(key);
      row.append(labelCell);
      const baseline = runs[0].summary?.[key];
      for (const [index, run] of runs.entries()) {
        const value = run.summary?.[key];
        const cell = document.createElement("td");
        const primary = document.createElement("strong");
        primary.textContent = summaryValue(key, value);
        cell.append(primary);
        if (index > 0 && typeof baseline === "number" && typeof value === "number") {
          const delta = value - baseline;
          const deltaText = document.createElement("small");
          deltaText.className = "metric-delta";
          deltaText.textContent = `Δ ${delta > 0 ? "+" : ""}${summaryValue(key, delta)}`;
          cell.append(deltaText);
        }
        row.append(cell);
      }
      elements.comparisonMetricsBody.append(row);
    }
  }

  function renderComparison(payload) {
    const runs = comparisonRunItems(payload);
    if (runs.length < 2 || runs.length > 4) throw new Error("比较接口必须返回 2–4 条运行结果。");
    if (runs.some((run) => run.model_id !== runs[0].model_id)) throw new Error("比较接口返回了不同模型的运行结果。");

    const altitudeSeries = runs.map((run) => ({
      points: finitePairs(run.series.time, run.series.altitude), color: run.color, dash: run.dash,
    }));
    const speedSeries = runs.map((run) => ({
      points: finitePairs(run.series.time, run.series.speed), color: run.color, dash: run.dash,
    }));
    renderOverlayChart(elements.comparisonAltitudeChart, altitudeSeries, { xLabel: "时间 (s)", yLabel: "高度 (m)" });
    renderOverlayChart(elements.comparisonSpeedChart, speedSeries, { xLabel: "时间 (s)", yLabel: "速度 (m/s)" });

    elements.comparisonLegend.replaceChildren();
    for (const [index, run] of runs.entries()) {
      const item = document.createElement("div");
      item.className = "legend-item";
      const swatch = document.createElement("span");
      swatch.className = "legend-line";
      swatch.style.setProperty("--legend-color", run.color);
      swatch.classList.toggle("is-dashed", index > 0);
      const text = document.createElement("span");
      const title = document.createElement("strong");
      const detail = document.createElement("small");
      title.textContent = index === 0 ? `${run.name}（基准）` : run.name;
      detail.textContent = `${run.run_id.slice(0, 12)} · ${formatRunConfiguration(run)}`;
      detail.title = formatRunConfiguration(run, false);
      text.append(title, detail);
      item.append(swatch, text);
      elements.comparisonLegend.append(item);
    }
    renderComparisonMetrics(runs);
    elements.comparisonEmpty.hidden = true;
    elements.comparisonContent.hidden = false;
    elements.comparisonCaption.textContent = `已加载 ${runs.length} 条 ${runs[0].model_id || "同模型"} 运行；模块组合可不同，首条为差值基准，曲线保留各自原生时间轴。`;
    elements.comparisonContent.scrollIntoView({ behavior: "smooth", block: "start" });
  }

  async function compareSelectedRuns() {
    const selected = selectedComparisonRuns();
    if (selected.length < 2 || selected.length > 4) return;
    if (!selected.every((run) => successfulHistoryRun(run) && run.model_id === selected[0].model_id)) {
      elements.comparisonCaption.textContent = "请选择 2–4 条同模型成功运行。";
      return;
    }
    showScreen("compare");
    hideError();
    state.comparisonRunning = true;
    elements.compareRuns.textContent = "正在比较…";
    elements.comparisonContent.hidden = true;
    elements.comparisonEmpty.hidden = false;
    elements.comparisonEmpty.querySelector("strong").textContent = "正在加载比较结果";
    elements.comparisonEmpty.querySelector("p").textContent = "服务端正在读取所选运行。";
    syncComparisonControls();
    try {
      const { payload } = await apiRequest("/comparisons", {
        method: "POST",
        body: JSON.stringify({ run_ids: selected.map((run) => run.run_id) }),
      });
      renderComparison(payload);
    } catch (error) {
      const message = `结果比较失败：${errorMessage(error)}`;
      clearComparisonView(message);
      elements.comparisonEmpty.querySelector("strong").textContent = "比较失败";
      elements.comparisonEmpty.querySelector("p").textContent = errorMessage(error);
      showError(message);
    } finally {
      state.comparisonRunning = false;
      elements.compareRuns.textContent = "执行比较";
      syncComparisonControls();
    }
  }

  function renderResult(result) {
    const samples = extractSamples(result);
    const series = {
      time: numericSeries(samples, "time"),
      altitude: numericSeries(samples, "altitude"),
      speed: numericSeries(samples, "speed"),
      east: numericSeries(samples, "east"),
      north: numericSeries(samples, "north"),
      longitude: numericSeries(samples, "longitude"),
      latitude: numericSeries(samples, "latitude"),
    };
    const altitudePoints = finitePairs(series.time, series.altitude);
    const speedPoints = finitePairs(series.time, series.speed);
    let trajectoryPoints = finitePairs(series.east, series.north);
    let trajectoryLabels = { x: "东向位置 (m)", y: "北向位置 (m)", caption: "东向 / 北向位置 (m)" };
    if (trajectoryPoints.length < 2) {
      trajectoryPoints = finitePairs(series.longitude, series.latitude);
      trajectoryLabels = { x: "经度 (°)", y: "纬度 (°)", caption: "经度 / 纬度 (°)" };
    }

    renderLineChart(elements.altitudeChart, altitudePoints, {
      xLabel: "时间 (s)", yLabel: "高度 (m)", seriesClass: "series-altitude",
    });
    renderLineChart(elements.speedChart, speedPoints, {
      xLabel: "时间 (s)", yLabel: "速度 (m/s)", seriesClass: "series-speed",
    });
    renderLineChart(elements.trajectoryChart, trajectoryPoints, {
      xLabel: trajectoryLabels.x,
      yLabel: trajectoryLabels.y,
      seriesClass: "series-trajectory",
      endpoints: true,
      height: 360,
    });
    elements.trajectoryCaption.textContent = trajectoryLabels.caption;

    const explicitSummary = result.summary && typeof result.summary === "object" ? result.summary : {};
    renderSummary({ ...derivedSummary(samples, series), ...explicitSummary });
    elements.resultsEmpty.hidden = true;
    elements.resultsContent.hidden = false;
    elements.resultActions.hidden = false;
    elements.resultsCaption.textContent = samples.length
      ? `已加载运行 ${state.runId} 的 ${samples.length} 个结果样本。`
      : `运行 ${state.runId} 已完成，但结果未包含可识别的时序样本。`;
    elements.resultsContent.scrollIntoView({ behavior: "smooth", block: "start" });
  }

  async function copyResult() {
    if (!state.latestResult) return;
    try {
      await navigator.clipboard.writeText(JSON.stringify(state.latestResult, null, 2));
      const original = elements.copyResult.textContent;
      elements.copyResult.textContent = "已复制";
      window.setTimeout(() => { elements.copyResult.textContent = original; }, 1600);
    } catch {
      showError("浏览器未授予剪贴板权限。可以从开发者工具的网络响应中保存结果。 ");
    }
  }

  function clearLogDisplay() {
    elements.logOutput.textContent = "日志显示已清空；新的服务端日志仍会继续出现。";
    state.logs = [];
    state.logSet.clear();
  }

  function bindEvents() {
    for (const item of elements.navItems) {
      item.addEventListener("click", () => showScreen(item.dataset.screenTarget));
    }
    elements.reloadCatalog.addEventListener("click", () => void loadModelCatalog());
    elements.runForm.addEventListener("submit", startRun);
    elements.reloadModels.addEventListener("click", loadModels);
    elements.modelSelect.addEventListener("change", updateModelHelp);
    elements.resetParameters.addEventListener("click", resetParameterValues);
    elements.cancelRun.addEventListener("click", cancelRun);
    elements.dismissError.addEventListener("click", hideError);
    elements.clearLogs.addEventListener("click", clearLogDisplay);
    elements.copyResult.addEventListener("click", copyResult);
    elements.reloadHistory.addEventListener("click", () => void loadHistory());
    elements.compareRuns.addEventListener("click", compareSelectedRuns);
    window.addEventListener("pagehide", () => {
      state.runToken += 1;
      stopPolling();
    }, { once: true });
  }

  bindEvents();
  showScreen("catalog");
  void loadModelCatalog();
  void loadModels();
  void loadHistory({ silent: true });
})();
