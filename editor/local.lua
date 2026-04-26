local function close_windows(name)
  -- Get a list of all windows
  local cmd = 'taskkill /IM ' .. name .. ' /F'
  local results = vim.fn.system(cmd)
  --print(results)
end

vim.keymap.set('n', '<F7>', 
function()
    close_windows("slicer.exe")
    local root_dir = vim.fn.getcwd()
    local start_cmd = 'start ' .. root_dir .. '/bin/slicer.exe '
    vim.fn.system(start_cmd)
end, { noremap = true, silent = true})

vim.keymap.set('n', '<F5>', 
function()
    close_windows("slicer.exe")
    local root_dir = vim.fn.getcwd()
    local build_cmd = 'start cmd /c "(cmake --build ' .. root_dir .. '/build/clang/ > build_output.txt 2>&1) & findstr /i /c:\"error\" build_output.txt && (pause) || del build_output.txt"'
    vim.fn.system(build_cmd)
end, { noremap = true, silent = true})

